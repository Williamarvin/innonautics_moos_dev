/************************************************************/
/*    NAME: Blake Cole                                      */
/*    ORGN: MIT                                             */
/*    FILE: matrix_playground.cpp                           */
/*    DATE: 30 MARCH 2021                                   */
/************************************************************/

// MATRIX_PLAYGROUND tests the capabilities of the armadillo
//   c++ library for linear algebra and scientific computing.

// References:
//   (1) "Documentation for Armadillo"
//       <http://arma.sourceforge.net/docs.html>

// Compile using the following command line switches:
// $ g++ -std=c++11 -O2 ./matrix_playground.cpp
//       -o matrix_playground
//       -Wall
//       -I/opt/local/include
//       -L/opt/local/lib -larmadillo
//       -DARMA_DONT_USE_WRAPPER
//       -framework Accelerate OR -lopenblas

#include <cmath>
#include <chrono>
#include <armadillo>   /* Linear algebra library */
#include <iostream>    /* Print to terminal      */

#define PI M_PI
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))
#define sind(x) (sin(fmod((x),360) * M_PI / 180))

#define matprint(name) printer(#name, (name))
void printer(std::string name, arma::mat value) {
  std::cout << name << " =\n" << value << std::endl;
}

using namespace std::chrono;
using dseconds = duration<double>;

// define typedefs for enhanced readability of complex lists/vectors
// in MOOS files, these typically live in the .h header files.
typedef std::pair<arma::vec, arma::mat> VECMAT_PAIR;
typedef std::chrono::system_clock CLOCK;
typedef std::chrono::milliseconds MILLIS;

// query variable type
// std::cout << "N = " <<N<< " [" << typeid(N).name()  << "]" << std::endl;

double mod(double a, double m) {
  if (m==0)
    return(a);
  a -= m * floor(a / m) + m;
  return(a - m * floor(a / m));
}

double round(const double &x, const int &n)
{
  double tens = pow(10,n);
  while (abs(x*tens) < 1)
    tens *= 10;
  double r = x*tens;
  return(floor(r + 0.5) / tens);
}

// ********************************************
// SIGMAS
VECMAT_PAIR sigmaPoints(arma::vec x, arma::mat P)
{
  // input error checking
  if (x.n_rows > 1 && x.n_cols > 1)
    std::cout << "ERROR: state vector cannot be a matrix." << std::endl;
  else if (x.n_cols > x.n_rows) {
    std::cout << "WARNING: state vector transposed." << std::endl;
    x.t();
  }
  
  if (!P.is_symmetric())
    std::cout << "ERROR: covariance matrix must be symmetric." << std::endl;
  else if (!(P.n_rows == x.n_rows))
    std::cout << "ERROR: covariance matrix must have the same number of rows and columns as the state vector has rows." << std::endl;

  // compute weights and generate corresponding sigma points
  double     N = x.n_rows;
  double     w0 = 1-(N/3);
  arma::vec  weights(2*N+1);
  weights.fill((1-w0)/(2*N));
  weights(0) = w0;

  arma::mat S = arma::chol((N/(1-w0))*P, "lower");
  arma::mat xcopies = arma::repmat(x,1,N);  //(nxn) matrix of (nx1) state copies
  arma::mat points = arma::join_horiz(x, xcopies+S, xcopies-S);
  
  return(std::make_pair(weights, points));
}


// ********************************************
// PREDICT
VECMAT_PAIR predict(arma::vec x,
                    arma::mat P,
                    arma::mat Q,
                    dseconds dt,
                    double a,
                    double r)
{
  VECMAT_PAIR sigmas = sigmaPoints(x, P);

  arma::rowvec lon1 = (sigmas.second.row(0))*(M_PI / 180);
  arma::rowvec lat1 = (sigmas.second.row(1))*(M_PI / 180);
  arma::rowvec u1   = sigmas.second.row(2);
  arma::rowvec az1  = sigmas.second.row(3);

  // update position with spherical law of cosines
  double       R = 6371e+03;
  arma::rowvec d = dt.count()*u1;
  arma::rowvec c = d/R;

  arma::rowvec lat2 = asin(sin(lat1)%cos(c) + cos(lat1)%sin(c)%cos(az1));
  arma::rowvec lon2 = lon1+atan2(sin(c)%sin(az1),
                                 cos(lat1)%cos(c) - sin(lat1)%sin(c)%cos(az1));
  lat2 *= (180 / M_PI);
  lon2 *= (180 / M_PI);

  // update speed and heading
  arma::rowvec u2  = u1 + a*dt.count();
  arma::rowvec az2 = az1 + r*dt.count();

  // a priori state
  arma::mat Y    = arma::join_vert(lon2, lat2, u2, az2);
  arma::vec xbar = arma::sum(Y.each_row()%sigmas.first.t(),1);

  // a priori covariance
  arma::mat Pbar = (Y.each_col()-xbar)* arma::diagmat(sigmas.first)*(Y.each_col()-xbar).t();
  Pbar += Q;

  matprint(xbar);
  matprint(Pbar);
  
  return (std::make_pair(xbar, Pbar));
}


// ********************************************
// CORRECT
VECMAT_PAIR correct(VECMAT_PAIR a_priori, arma::vec z, arma::mat H, arma::mat R)
{
  arma::vec xbar = a_priori.first;
  arma::mat Pbar = a_priori.second;

  arma::mat K = Pbar*H.t() * arma::inv(H*Pbar*H.t() + R);
  matprint(K);
  arma::vec y = z - H*xbar;
  matprint(y);

  if (K.has_nan()) {
    std::cout << "ERROR: K is nan bruh." << std::endl;
  }

  // account for course/heading discontinuity at 360 degrees
  y(3) = mod(y(3)+180,360) - 180;
  arma::vec x = xbar + K*y;
  x(3) = mod(x(3),360);
  
  // Joseph form
  arma::mat P = (arma::eye(4,4)-K*H)*Pbar*(arma::eye(4,4)-K*H).t() + K*R*K.t();

  matprint(x);
  matprint(P);
  return (std::make_pair(x, P));
}
  
//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//
int main()
{
  std::cout << "**********************************" << std::endl;
  std::cout << "----------------------------------" << std::endl;
  std::cout << "Armadillo UKF Sandbox           \n" << std::endl;

  std::cout << "----------------------------------" << std::endl;
  std::cout << "Misc. Preliminaries               " << std::endl;
  std::cout << "cosd(-90) = " << cosd(-90) << std::endl;
  std::cout << "sind(-90) = " << sind(-90) << "\n" << std::endl;

  CLOCK::time_point tp1 = CLOCK::now();
  std::cout << "current time = " << tp1.time_since_epoch().count() << std::endl;

  double testval = -321.1234;
  std::cout << "testval = " << testval << std::endl;
  std::cout << "round(testval,-4) = " << round(testval,-4) << std::endl;
  std::cout << "round(testval,-3) = " << round(testval,-3) << std::endl;
  std::cout << "round(testval,-2) = " << round(testval,-2) << std::endl;
  std::cout << "round(testval,-1) = " << round(testval,-1) << std::endl;
  std::cout << "round(testval,0) = " << round(testval,0)  << std::endl;
  std::cout << "round(testval,1) = " << round(testval,1)  << std::endl;
  std::cout << "round(testval,2) = " << round(testval,2)  << std::endl;
  std::cout << "round(testval,3) = " << round(testval,3)  << std::endl;
  std::cout << "round(testval,4) = " << round(testval,4)  << std::endl;
  std::cout << "round(testval,5) = " << round(testval,5)  << std::endl;

  std::cout << "\nHow does a single iterator move through matrix?" << std::endl;
  arma::mat M = arma::repmat(arma::linspace<arma::vec>(1,4,4), 1, 5);
  matprint(M);
  for (int i=0; i<M.n_elem; ++i) {
    std::cout << M(i) << ",";
  }
  std::cout << std::endl;
  
  std::cout << "----------------------------------" << std::endl;
  std::cout << "Initialize Filter                 " << std::endl;
  //double dt = 2.0;
  dseconds dt{2.0};
  
  // dynamic variables
  double  lon = -71.0311;
  double  lat = 42.3623;
  double  sog = 0.0514444;
  double  cog = 355.9;
  arma::vec::fixed<4> x = {lon, lat, sog, cog};
  matprint(x);
  
  // initial state variance
  double  var_lon = 1e-6;
  double  var_lat = 1e-6;
  double  var_sog = 4;
  double  var_cog = 36;

  // initial state covariance matrix
  arma::vec::fixed<4> v = {var_lon, var_lat, var_sog, var_cog};
  arma::mat::fixed<4,4> P = arma::diagmat(v);
  matprint(P);

  // initial residual error
  arma::vec::fixed<4> y(arma::fill::zeros);
  matprint(y);

  // compute length of one degree lon & lat
  // <https://en.wikipedia.org/wiki/Latitude#Length_of_a_degree_of_latitude>
  double lon_len, lat_len, aa, e2;
  aa = 6378137;            //equatorial radius [m]
  e2 = 0.00669437999014;   //eccentricity squared
  lat_len = 111132.954 - 559.822*cosd(2*lat) + 1.175*cosd(4*lat);
  lon_len = PI*aa*cosd(lat) / (180*sqrt(1 - pow(e2*sind(lat),2)));
  std::cout << "At " << lat << " degrees latitude," << std::endl;
  std::cout << "  1 deg longitude = " << lon_len << " m" << std::endl;
  std::cout << "  1 deg latitude  = " << lat_len << " m\n" << std::endl;
        
  // initial proccess noise covariance matrix
  double zeta = 5.0;
  double sdev_lon = zeta / lon_len;
  double sdev_lat = zeta / lat_len;
  double sdev_sog = 0.1;
  double sdev_cog = 1.6;
  v = {sdev_lon, sdev_lat, sdev_sog, sdev_cog};
  
  arma::mat::fixed<4,4> Q(arma::fill::zeros);
  Q(0,0) = pow(sdev_lon,2)*pow(dt.count(),2);
  Q(1,1) = pow(sdev_lat,2)*pow(dt.count(),2);
  Q(2,2) = pow(sdev_sog,2)*dt.count();
  Q(3,3) = pow(sdev_cog,2)*dt.count();
  Q(2,0) = pow(sdev_lon*sind(cog),2)*dt.count();
  Q(2,1) = pow(sdev_lat*cosd(cog),2)*dt.count();
  Q += arma::trimatl(Q, -1).st();

  matprint(Q);
  // check for symmetric positive definite
  if (Q.is_sympd())
    std::cout << "Q is symmetric positive definite.\n" << std::endl;
  else {
    std::cout << "Q IS NOT symmetric positive definite." << std::endl;
    std::cout << "Deleting off-diagonal terms...\n" << std::endl;
    Q = diagmat(Q);
    matprint(Q);
  }
  
  // initial measurement noise covariance matrix
  // <https://www.advancednavigation.com/products/gnss-compass>
  double gnss_horiz_acc = 2.0;  //95% CEP [m]
  v = {gnss_horiz_acc/lon_len, gnss_horiz_acc/lat_len, 0.05, 0.2};
  arma::mat::fixed<4,4> R = diagmat(pow(v,2));
  matprint(R);

  arma::SizeMat size_R = size(R);
  std::cout << "size of R: " << size_R << "\n" << std::endl;

  std::cout << "----------------------------------" << std::endl;
  std::cout << "Predict                           " << std::endl;
  
  double a = 0;  // acceleration = 0
  double r = 0;  // turn rate = 0
  VECMAT_PAIR a_priori = predict(x, P, Q, dt, a, r);
  
  std::cout << "----------------------------------" << std::endl;
  std::cout << "Correct                           " << std::endl;
  
  arma::vec::fixed<4> z = {-71.03, 42.362, 0.04, 10};
  arma::mat::fixed<4,4> H(arma::fill::eye);
  matprint(z);
  matprint(H);
  VECMAT_PAIR a_posteriori = correct(a_priori, z, H, R);

  // get total runtime
  CLOCK::time_point tp2 = CLOCK::now();
  std::cout << "total run time = ";
  std::cout << duration_cast<microseconds>(tp2 - tp1).count() << "us [";
  std::cout << dseconds{tp2 - tp1}.count() << "s]\n" << std::endl;

  // test mod stuff
  std::cout << "----------------------------------" << std::endl;
  std::cout << "Test MOD stuff                      " << std::endl;

  double y_raw = -187.22;
  double y_fix = mod(y_raw+180,360) - 180;
  std::cout << "y_raw = " << y_raw << std::endl;
  std::cout << "y_fix = " << y_fix << "\n" << std::endl;

  double x_raw = 530.3429;
  double x_fix = mod(x_raw,360);
  std::cout << "x_raw = " << x_raw << std::endl;
  std::cout << "x_fix = " << x_fix << std::endl;
  
  return(0);
}
