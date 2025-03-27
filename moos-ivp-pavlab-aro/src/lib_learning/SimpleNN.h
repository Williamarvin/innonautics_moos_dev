/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleNN.h                                      */
/*    DATE: June 29th, 2021                                 */
/************************************************************/

#ifndef SimpleNN_HEADER
#define SimpleNN_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <armadillo>
#include <cmath>  // for abs() overloads

using namespace arma;


class SimpleNN
{
 public:
  SimpleNN();
  ~SimpleNN();

  bool   initialize(long long unsigned int num_of_neurons, long long unsigned int num_of_inputs);
  bool   setWeights(Mat<double> W, Col<double> B, Col<double> A);
  bool   getWeights(Mat<double> &W, Col<double> &B, Col<double> &A);
  bool   setLearningRateAdam(double alpha, double beta1, double beta2, double epsilon);
  bool   setLearningRateContraction(double c_w) {m_contraction_weight = c_w; return(true);};
  bool   setUseContraction(bool in);
  double forwardProp(std::vector<double> inputs);
  bool   backPropAdam(double error, std::vector<double> inputs);
  bool   backPropRecurrentAdam(double error_t1, std::vector<double> inputs_t1, double error_t2, std::vector<double> inputs_t2);

  bool   isRNNContracting();
  std::vector<double> getEqualibPoints();
  
 protected:  
  bool   updateWeightsAdam(Mat<double> grad_W, Col<double> grad_B, Col<double> grad_A);
  bool   updateContractionLossGrad();
 
  Col<double> activationFun(Col<double> in);
  Col<double> activationFunDerivative(Col<double> in);

  bool   m_contracting;    // use the contracting cost or not.

  double m_alpha;
  double m_beta1;
  double m_beta2;
  double m_epsilon;
  double m_contraction_weight;

  uint m_num_back_props;    // To adjust the learning rate alpha
  
  uword   m_number_of_neurons;
  uword   m_number_of_inputs;
  //double m_number_of_outputs;  TODO add multiple outpus

  Mat<double> m_W;    // Matrix of weights for inputs to hidden layer
  Col<double> m_B;    // Vector of bias terms for each neuron in hidden layer
  Col<double> m_A;    // Vector of weights for the output from the hidden layer 
  Col<double> m_Z;    // Vector of inputs to each neuron in hidden layer
  Col<double> m_G;    // Vector of outpus from each neuron in hidden layer

  Mat<double> m_W_grad_cont;  // Matrix to hold any contraction errors for weights in W
  Col<double> m_A_grad_cont;  // Vector to hold any contraction errors for weights in A

  // Matrix and Vectors to hold the first and second moment info for Adam
  Mat<double> m_mom1_W;    // Matrix of first moments for W
  Mat<double> m_mom2_W;    // Matrix of second moments for W
  Col<double> m_mom1_B;    // Vector of first moments for B
  Col<double> m_mom2_B;    // Vector of second moments for B
  Col<double> m_mom1_A;    // Vector of first moments for A
  Col<double> m_mom2_A;    // Vector of second moments for A
  

 private:

};
  










#endif 
