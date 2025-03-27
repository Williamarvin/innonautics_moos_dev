/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleAID.cpp                                    */
/*    DATE: June 29th, 2021                                 */
/************************************************************/

/* This class defines a simple scalar adaptive identifier 
   (AID) to perform recurrent estimation for a 1-DOF dynamical 
   system. i.e. v_dot = f(v,u).  This approach is described in 
   this paper: https://ieeexplore.ieee.org/abstract/document/1208328
   and also draws upon insights reported in other papers from
   the DSCL at Johns Hopkins. 
*/



#include "SimpleAID.h"

using namespace arma;

//---------------------------------------------------------
// Constructor

SimpleAID::SimpleAID()
{
  m_number_of_params = 1;
  m_a_m = 0;

  m_v_hat = 0;
  m_v_hat_dot = 0;
}


//---------------------------------------------------------
// Destructor

SimpleAID::~SimpleAID()
{
}

//-----------------------------------------------------------
// Initialize

bool SimpleAID::initialize(long long unsigned int num_of_params)
{
  m_number_of_params = num_of_params; // "convert" from unsigned int to uword
  m_Gamma = zeros(m_number_of_params, m_number_of_params);

  m_phi_hat = zeros(m_number_of_params);

  return true;
}

//--------------------------------------------------------------
// Set params
bool   SimpleAID::setParams(std::vector<double> phi_hat)
{
  m_phi_hat = conv_to< Col<double> >::from(phi_hat);
  return true;
}


//---------------------------------------------------------
// Get current params

std::vector<double> SimpleAID::getParams()
{
  std::vector<double> params = conv_to< std::vector<double> >::from(m_phi_hat);
  return params;
}



//---------------------------------------------------------z
// Set adaptation gains

bool SimpleAID::setAdaptationGains(std::vector<double> Gamma, double a_m)
{
  // check size  // Both return a uint
  if (m_number_of_params != Gamma.size() ) {
    return false;
  }

  // Convert to arma:Col
  Col<double> Gamma_col = conv_to< Col<double> >::from(Gamma);
  
  m_Gamma.diag() = Gamma_col;
  m_a_m = a_m;

  return true;
}




//--------------------------------------------------------
// Generate current estimate of the time derivative of
// velocity using the current estimate of the parameters
// Inputs f_hat -> state vector
// Returns a scalar, v_dot
double SimpleAID::estVelDot(std::vector<double> f_hat)
{

  // Convert to arma:Col
  Col<double> f_hat_col = conv_to< Col<double> >::from(f_hat);

  double v_dot = dot(m_phi_hat, f_hat_col) / 3.0;
  
  return v_dot;
  
}


//------------------------------------------------------
// Adaptively update the parameters from the error delta_v
// and the vector f_hat
// Inputs: The mesured velocity, "meas_v", taken at time
// "dt" after the previous measurement was provided.
// f_hat is the state vector at the previous measurement
// - it is the state that resulted in the new measured vel

bool SimpleAID::updateParams(double meas_v, double dt, std::vector<double> f_hat)
{

  // Convert to arma:Col
  Col<double> f_hat_col = conv_to< Col<double> >::from(f_hat);

  // Update the AID plant estimate using the last time step
  m_v_hat = m_v_hat + m_v_hat_dot * dt;

  // Calculate Delta V
  double delta_v = m_v_hat - meas_v;
  
  // Calculate v_hat_dot for this time step
  m_v_hat_dot = m_a_m * delta_v + dot(m_phi_hat, f_hat_col);

  // Update the parameters
  Col<double> phi_hat_dot = - delta_v * m_Gamma * f_hat_col;
  m_phi_hat = m_phi_hat + phi_hat_dot * dt;

  // Normalize to keep the parameter estimate on the unit sphere
  m_phi_hat = arma::normalise(m_phi_hat);

  // Bound the drag terms to be on the right half plane (positive).
  // This effectively restricts the solution to a hemisphere of the
  // unit sphere.  This is a practical fix to avoid positive gains
  // that might occur if the system is not well modeled and/or
  // parameterized.
  double epsilon = .00000001;
  if ( m_phi_hat(2) > 0.0 ) 
    m_phi_hat(2) = - epsilon;
  if (m_phi_hat(3) > 0.0 ) 
    m_phi_hat(3) = - epsilon;
  
  return true;
  
}


