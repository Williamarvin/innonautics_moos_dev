/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleNN.h                                      */
/*    DATE: June 29th, 2021                                 */
/************************************************************/

#ifndef SimpleAID_HEADER
#define SimpleAID_HEADER


#include <armadillo>

using namespace arma;

class SimpleAID
{
 public:
  SimpleAID();
  ~SimpleAID();

  bool   initialize(long long unsigned int num_of_params);
  bool   setParams(std::vector<double> phi_hat);
  std::vector<double>   getParams();
  bool   setAdaptationGains(std::vector<double> Gamma, double a_m);
  bool   setInitialvHat(double v) {m_v_hat = v; return true;};
  double estVelDot(std::vector<double> f_hat);
  bool   updateParams(double meas_v, double dt, std::vector<double> f_hat);
 

 protected:
  uword   m_number_of_params;
  Mat<double> m_Gamma;    // Adaptation Gain Matrix (must be diagonal
                          // and positive definite)
  double m_a_m;           // Scalar adaptation gain
  Col<double> m_phi_hat;  // Vector of current parameter estimates

  double m_v_hat;         // Current estimate of the velocity,
                          // the state of the identifier plant
  double m_v_hat_dot;     // Current estimate of the derivative
                          // of the state of the ID plant
  



  

 private:

};
  










#endif 
