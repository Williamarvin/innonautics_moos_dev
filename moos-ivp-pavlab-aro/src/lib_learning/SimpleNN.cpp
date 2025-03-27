/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleNN.cpp                                    */
/*    DATE: June 29th, 2021                                 */
/************************************************************/

/* A simple shallow network class object to estimate any 
   function.  Uses Adam for back propagation and a sigmoid 
   activation for estimation.  The inputs must be scaled 
   to the range [0,1] (inclusive) and the outputs will 
   also be in the range (0,1). 

   Also includes a method to perform recurrent estimation
   for a 1-DOF dynamical system. i.e. v_dot = f(v,u). 
   The input vector to the NN is assumed to have the state 
   variable (v) in the first entry of the vector. 
*/



#include "SimpleNN.h"

using namespace arma;

//---------------------------------------------------------
// Constructor

SimpleNN::SimpleNN()
{
   // set the seed to a random value
  arma_rng::set_seed_random();

  m_number_of_neurons = 10;
  m_number_of_inputs  = 0;
  m_num_back_props    = 0;

  m_alpha = 0.0;
  m_beta1 = 0.0;
  m_beta2 = 0.0;
  m_epsilon = 0.0;
  m_contraction_weight = 0.0;

  m_contracting = false;
}


//---------------------------------------------------------
// Destructor

SimpleNN::~SimpleNN()
{
}


//-----------------------------------------------------------
// Initialize
bool SimpleNN::initialize(long long unsigned int num_of_neurons, long long unsigned int num_of_inputs)
{
  m_number_of_neurons = num_of_neurons;  // "convert" from unsigned int to uword
  m_number_of_inputs  = num_of_inputs;   // "convert" from unsigned int to uword

  // Initialize weights in matrix W and vectors B, and A
  // Initialize to be random for now.  These can be reset with
  // pretrained weights later
  m_W.randu(m_number_of_neurons, m_number_of_inputs);
  
  m_B.randu(m_number_of_neurons);
  m_A.randu(m_number_of_neurons);

  // Initialize the contraction-based gradient matrix and vector
  m_W_grad_cont = zeros(m_number_of_neurons, m_number_of_inputs);
  m_A_grad_cont = zeros(m_number_of_neurons);

  // Initialize Z and G, the vectors containing the inputs and the
  // outputs of the nuerons (respectively).

  m_Z.zeros(m_number_of_neurons);
  m_G.zeros(m_number_of_neurons);

  // Initialize the first and second moment matrix and vectors
  m_mom1_W = zeros(m_number_of_neurons, m_number_of_inputs);
  m_mom2_W = zeros(m_number_of_neurons, m_number_of_inputs);
  m_mom1_B = zeros(m_number_of_neurons);
  m_mom2_B = zeros(m_number_of_neurons);
  m_mom1_A = zeros(m_number_of_neurons);
  m_mom2_A = zeros(m_number_of_neurons);

  return true;
}



//---------------------------------------------------------
// Set weights of network

bool SimpleNN::setWeights(Mat<double> W, Col<double> B, Col<double> A)
{

  // Check sizes before setting
  if ( (W.n_cols == m_W.n_cols) && (W.n_rows == m_W.n_rows) ) {
    m_W = W; 
  }
  else {
    return false;
  }

  if ( (B.n_cols == m_B.n_cols) && (B.n_rows == m_B.n_rows) ) {
    m_B = B; 
  }
  else {
    return false;
  }

  if ( (A.n_cols == m_A.n_cols) && (A.n_rows == m_A.n_rows) ) {
    m_A = A; 
  }
  else {
    return false;
  }

  return true;
}


//---------------------------------------------------------
// Get weights of network

bool SimpleNN::getWeights(Mat<double> &W, Col<double> &B, Col<double> &A)
{
  // Check sizes before returning
  if ( (W.n_cols == m_W.n_cols) && (W.n_rows == m_W.n_rows) ) {
    W = m_W; 
  }
  else {
    return false;
  }
  
  if ( (B.n_cols == m_B.n_cols) && (B.n_rows == m_B.n_rows) ) {
    B = m_B; 
  }
  else {
    return false;
  }

  if ( (A.n_cols == m_A.n_cols) && (A.n_rows == m_A.n_rows) ) {
    A = m_A; 
  }
  else {
    return false;
  }
  
  return true;
 
}


//--------------------------------------------------------------
// Set learning Rates
bool   SimpleNN::setLearningRateAdam(double alpha, double beta1, double beta2, double epsilon)
{
  m_alpha = alpha;
  m_beta1 = beta1;
  m_beta2 = beta2;
  m_epsilon = epsilon;

  return true;
}



//--------------------------------------------------------
// Forward Propagate
// Input is a std::vector of input values, all scaled from
// 0 to 1.  std::vector is used instead of arma::Col to reduce
// dependencies when this class is used.
// Returns the value at the output node, which is scaled from
// 0 to 1, or -1 if error
double SimpleNN::forwardProp(std::vector<double> inputs)
{
  // check size  // Both return a uint
  if (m_W.n_cols != inputs.size() ) {
    return -1;
  }

  // Convert to arma:Col
  Col<double> input_col = conv_to< Col<double> >::from(inputs);

  // Propagate from input to output
  m_Z = m_W * input_col + m_B;
  m_G = activationFun(m_Z);
  double val_at_output = dot(m_G, m_A);

  // Apply activation function again. This is built for the
  // multiple output case - to be done later.
  uint num_outputs = 1;
  Col<double> vals_at_out_layer_col(num_outputs);
  vals_at_out_layer_col(0) = val_at_output;
  
  Col<double> Outputs = activationFun(vals_at_out_layer_col);
  // Only one output for now
  double NNoutput = Outputs(0);
 
  return NNoutput;
  
}


//------------------------------------------------------
// Back propagate error using Adam proceedure
// Return true if sucessful
// std::vector is again used instead of arma::Col to reduce
// dependencies.

bool SimpleNN::backPropAdam(double error, std::vector<double> inputs)
{

  // check size  // Both return a uint
  if (m_W.n_cols != inputs.size() ) {
    return false;
  }

  // Convert to arma:Col
  Col<double> input_col = conv_to< Col<double> >::from(inputs);

  // Forward propagate to be sure that all member matrices are up
  // to date
  double output_fp = this->forwardProp(inputs);



  // Do some preliminary math we will use repeatedly
  // Again this is partially built for the multiple output
  // case - to be done later
  uint num_outputs = 1;
  Col<double> tempC(num_outputs);
  tempC(0) = dot(m_G, m_A);
  
  Col<double> g_prime_G_dot_A(num_outputs);
  g_prime_G_dot_A = activationFunDerivative( tempC );
  
  double g_prime_G_dot_A_dble = g_prime_G_dot_A(0);

  
  // Calculate the gradient for A
  Col<double> grad_A =  error * g_prime_G_dot_A_dble * m_G;

  // Calculate the gradient for the bias in each neuron
  Col<double> grad_B =  error * g_prime_G_dot_A_dble * m_A % activationFunDerivative(m_Z);

  // Calculate the gradient for the inputs on the hidden layer 
  Mat<double> grad_W = kron( grad_B, input_col.t() );

  // Update weights using Adam
  bool ok = this->updateWeightsAdam(grad_W, grad_B, grad_A);

  return ok;
  
}



//------------------------------------------------------
// Back propagate error in a recurrent neural network using Adam
// Return true if sucessful
// std::vector is again used instead of arma::Col to reduce
// dependencies.
// The input vector to the NN is assumed to have the state 
// variable (v) in the first entry of the vector.
// t1 is time step 1 where the associated error is error_t1 (which
// is really the error at timestep 2), and the same for t2. t2>t1.

bool   SimpleNN::backPropRecurrentAdam(double error_t1, std::vector<double> inputs_t1, double error_t2, std::vector<double> inputs_t2) {

  // check size  // Both return a uint
  if  ( (m_W.n_cols != inputs_t1.size() ) || (m_W.n_cols != inputs_t2.size() ) ) {
    return false;
  }

  // Convert to arma:Col
  Col<double> input_col_t1 = conv_to< Col<double> >::from(inputs_t1);
  Col<double> input_col_t2 = conv_to< Col<double> >::from(inputs_t2);

 
  // Step 1.  Calculate grads at timestep 2
  
  // Forward propagate to time step 2 to be sure that all member
  // matrices are up to date
  double output_fp2 = this->forwardProp(inputs_t2);

  // Do some preliminary math we will use repeatedly
  // Again this is partially built for the multiple output
  // case - to be done later
  uint num_outputs = 1;
  Col<double> tempC(num_outputs);
  tempC(0) = dot(m_G, m_A);
  
  Col<double> g_prime_G_dot_A_t2(num_outputs);
  g_prime_G_dot_A_t2 = activationFunDerivative( tempC );
  double g_prime_G_dot_A_dble_t2 = g_prime_G_dot_A_t2(0);

  // Calculate the gradient of A at this timestep with this error
  Col<double> grad_A_error_t2 = error_t2 * g_prime_G_dot_A_dble_t2 * m_G;
  // Calculate the gradient of B at this timestep with this error
  Col<double> grad_B_error_t2 = error_t2 * g_prime_G_dot_A_dble_t2 * m_A % activationFunDerivative(m_Z);
  // Calculate the gradient of W at this timestep with this error
  Mat<double> grad_W_error_t2 = kron( grad_B_error_t2, input_col_t2.t() );       
            
  // Calculate the gradient of the error with respect to the state
  // variable (which is the recurrent state).  Assumed that the
  // state variable is in the first entry of the vector - the
  // zeroth column in armadillo
  double error_3 = dot(grad_B_error_t2, m_W.col(0) );


  // Step 2.  Calculate grads at the "second level" of the RNN
  // this includes the error at timestep t1 AND the recurrent
  // error, error_3 from timestep t2

  // Forward propagate to time step 1 to be sure that all member
  // matrices are up to date
  double output_fp1 = this->forwardProp(inputs_t1);

  // Do some preliminary math we will use repeatedly
  // Again this is partially built for the multiple output
  // case - to be done later
  tempC(0) = dot(m_G, m_A);
  Col<double> g_prime_G_dot_A_t1(num_outputs);
  g_prime_G_dot_A_t1 = activationFunDerivative( tempC );
  double g_prime_G_dot_A_dble_t1 = g_prime_G_dot_A_t1(0);

  // Calculate the gradient of A at this timestep with this error
  Col<double> grad_A_error_t1 = error_t1 * g_prime_G_dot_A_dble_t1 * m_G;
  // Calculate the gradient of B at this timestep with this error
  Col<double> grad_B_error_t1 = error_t1 * g_prime_G_dot_A_dble_t1 * m_A % activationFunDerivative(m_Z);
  // Calculate the gradient of W at this timestep with this error
  Mat<double> grad_W_error_t1 = kron( grad_B_error_t1, input_col_t1.t() );  
  

  // Unroll the RNN and calculate the gradients at the first
  // timestep from error_3. 
  // Calculate the gradient of A at this timestep with this error
  Col<double> grad_A_error_3 = error_3 * g_prime_G_dot_A_dble_t1 * m_G;
  // Calculate the gradient of B at this timestep with this error
  Col<double> grad_B_error_3 = error_3 *  g_prime_G_dot_A_dble_t1 * m_A % activationFunDerivative(m_Z);
  // Calculate the gradient of W at this timestep with this error
  Mat<double> grad_W_error_3 = kron( grad_B_error_3, input_col_t1.t() );


  // Step 3.  Combine all weights
  // Sum up all the gradients
  Col<double> grad_A = grad_A_error_t2 + grad_A_error_t1 + grad_A_error_3;
  Col<double> grad_B = grad_B_error_t2 + grad_B_error_t1 + grad_B_error_3;
  Mat<double> grad_W = grad_W_error_t2 + grad_W_error_t1 + grad_W_error_3;

  bool ok1 = true;
  // Step 3.5 Include any contraction error gradients
  if (m_contracting) {
    ok1 = updateContractionLossGrad();
    grad_W = grad_W + m_W_grad_cont;
    grad_A = grad_A + m_A_grad_cont;
  }
  
  // Step 4.  Update weights using Adam
  bool ok2 = this->updateWeightsAdam(grad_W, grad_B, grad_A);

  bool all_good = (ok1 and ok2);
  return all_good;
}



//-------------------------------------------------------------------------
// Update weights using Adam
// Refs: https://srdas.github.io/DLBook/GradientDescentTechniques.html#ParameterUpdate
//       https://machinelearningmastery.com/adam-optimization-from-scratch/
// Returns true if successful

bool   SimpleNN::updateWeightsAdam(Mat<double> grad_W, Col<double> grad_B, Col<double> grad_A){

  
  // Adjust the learning rate based on the epoch number
  double epoch_numb = floor(m_num_back_props / 1000.);
  double alpha_w_decay = m_alpha / sqrt( epoch_numb + 1.0);


  // Update the weights for the inputs on the hidden layer    
  // Calculate the first moment
  m_mom1_W = m_beta1 * m_mom1_W + (1.0 - m_beta1) * grad_W;
  // Calculate the second moment
  m_mom2_W = m_beta2 * m_mom2_W + (1.0 - m_beta2) * grad_W % grad_W;
  // Normalize the first and second moments
  Mat<double> mom1_W_hat = m_mom1_W / (1.0 - pow( m_beta1, (m_num_back_props + 1) ) );
  Mat<double> mom2_W_hat = m_mom2_W / (1.0 - pow( m_beta2, (m_num_back_props + 1) ) );
            
  //update W using Adam
  m_W = m_W - alpha_w_decay * mom1_W_hat / (arma::sqrt(mom2_W_hat) + m_epsilon);


  // Update the bias terms in each neuron
  // Calculate the first moment
  m_mom1_B = m_beta1 * m_mom1_B + (1.0 - m_beta1) * grad_B;
  // Calculate the second moment
  m_mom2_B = m_beta2 * m_mom2_B + (1.0 - m_beta2) * grad_B % grad_B;
  // Normalize the first and second moments
  Col<double> mom1_B_hat = m_mom1_B / (1.0 - pow( m_beta1, (m_num_back_props + 1) ) );
  Col<double> mom2_B_hat = m_mom2_B / (1.0 - pow( m_beta2, (m_num_back_props + 1) ) );
            
  //update B using Adam
  m_B = m_B - alpha_w_decay * mom1_B_hat / (arma::sqrt(mom2_B_hat) + m_epsilon);


  // Update the weights on the output of the hidden layer
  // Calculate the first moment
  m_mom1_A = m_beta1 * m_mom1_A + (1.0 - m_beta1) * grad_A;
  // Calculate the second moment
  m_mom2_A = m_beta2 * m_mom2_A + (1.0 - m_beta2) * grad_A % grad_A;
  // Normalize the first and second moments
  Col<double> mom1_A_hat = m_mom1_A / (1.0 - pow( m_beta1, (m_num_back_props + 1) ) );
  Col<double> mom2_A_hat = m_mom2_A / (1.0 - pow( m_beta2, (m_num_back_props + 1) ) );
            
  //update A using Adam
  m_A = m_A - alpha_w_decay * mom1_A_hat / (arma::sqrt(mom2_A_hat) + m_epsilon);
  

  
  m_num_back_props = m_num_back_props + 1.0;

  return true;
}




//--------------------------------------------------------------
// Activation Function
// Apply activation for each entry in the vector
// In this case it is the sigmoid function. 

Col<double> SimpleNN::activationFun(Col<double> in) {

  Col<double> output(in.n_elem);
  output.zeros();
  
  for(arma::vec::iterator it = in.begin(); it != in.end(); ++it){
    int index = std::distance(in.begin(), it);
    double val = *it;
    //  Actual Function definition here::

    // Sigmoid:
    //output(index) = 1 / (1 + std::exp(-val) );
    
    //RELU:
    if (val > 0) {
      output(index) = val;
    }
    
  }

  return output;
}


//--------------------------------------------------------------
// Activation Function Derivative
// Apply derivative of the activation for each entry in the vector
// In this case it is the sigmoid function. 

Col<double> SimpleNN::activationFunDerivative(Col<double> in) {

  // Derivative of the Signmoid function 
  /*
  Col<double> temp(in.n_elem);
  temp = activationFun(in);

  Col<double> col_ones(in.n_elem);
  col_ones.ones();
  
  Col<double> output(in.n_elem);
  output = temp % (col_ones - temp);
  */
  
  // Derivative of the RELU function
  Col<double> output(in.n_elem);
  output.zeros();
  output.elem( find(in > 0) ).ones();
  
  return output;
}


//-------------------------------------------------------------
// setUseContraction
// sets the boolean, and if needed, clears the matrices
bool SimpleNN::setUseContraction(bool in)   {
  
  m_contracting = in;

  if (in == false) {
    m_W_grad_cont = zeros(m_number_of_neurons, m_number_of_inputs);
    m_A_grad_cont = zeros(m_number_of_neurons);
  }

  return(true);
}


//------------------------------------------------------------
// updateContractionLossGrad()
// calculates the gradient of the contraction-based terms in the
// loss function and updates the member variables.
bool SimpleNN::updateContractionLossGrad()  {

  // check each neuron
  double numb_neurons = static_cast<double>(m_number_of_neurons);
  for (unsigned int i=0; i<m_number_of_neurons; i++) {
    double term1 = abs(m_A(i) + ( 2.0 / ( numb_neurons + 1.0 ) ) * abs(m_W(i,0)) );
    double term2 = 2.0 / ( numb_neurons + 1.0);

    if (term1 >= term2) {
      m_W_grad_cont(i,0) = m_contraction_weight * m_W(i,0);
      m_A_grad_cont(i)   = m_contraction_weight * m_A(i);
    } else {
      // clear
      m_W_grad_cont(i,1) = 0.0;
      m_A_grad_cont(i)   = 0.0;
    }
  }

  return(true);
}


//----------------------------------------------------------
// isRNNContracting()
// calculates if all neurons in the RNN are contracting
bool SimpleNN::isRNNContracting()  {

  // check each neuron
  double numb_neurons = static_cast<double>(m_number_of_neurons);
  for (unsigned int i=0; i<m_number_of_neurons; i++) {
    double term1 = abs(m_A(i) + ( 2.0 / ( numb_neurons + 1.0 ) ) * abs(m_W(i,0)) );
    double term2 = 2.0 / ( numb_neurons + 1.0);
    if (term1 >= term2) {
      return(false);
    }
  }
  // otherwise
  return(true);
}

//----------------------------------------------------------
// getEqualibPoints()
// first computes the critical points and checks for equalibrium
// between the critical points.
std::vector<double> SimpleNN::getEqualibPoints()  {

  // Step 1.  Get the critical points in [0,1], and order from smallest
  //          to largest - which happens in the set by default per cpp standard
  std::set<double>  critical_points;
  critical_points.insert(0.0);
  critical_points.insert(1.0);

  for (unsigned int i=0; i<m_number_of_neurons; i++) {
    double val = -m_B(i) / m_W(i,0);
    if ( (0 < val) and (val < 1) )
      critical_points.insert(val);
  }

  // Step 2.  Determine if the state response line crosses the equalib
  //          line

  // save time by calculating the y values for every x value in the set
  // the set is ordered, so each vector of x and y values will be ordered
  std::set<double>::iterator itr;
  std::vector<double> x_vals;
  std::vector<double> y_vals;
  for (itr = critical_points.begin(); itr != critical_points.end(); itr++) {
    
    x_vals.push_back(*itr);

    std::vector<double> vec;
    vec.push_back(*itr);
    for (int i=1; i<m_number_of_inputs; i++) {
      vec.push_back(0.0);
    }

    double y = forwardProp(vec);
    y_vals.push_back(y);
  }
  
  std::vector<double> eq_points;

  // We will always have at least two entries in the set, and the set
  // (and now vectors) will be ordered

  for (unsigned int i=0;  i<(x_vals.size()-1); i++) {
    
    double x1 = x_vals[i];
    double x2 = x_vals[i+1];

    double y1 = y_vals[i];
    double y2 = y_vals[i+1];

    // check if this segment crosses the equalib line
    //
    //  x1,y1   x2,x2
    //    .    / equalib line
    //     \  /
    //      \/
    //      /\
    //     /  \  
    //    /    \
    //   /      .
    //  x1,x1   x2, y2

    bool above1 = y1 > x1;
    bool above2 = y2 > x2;
    if ( (above1 and not above2) or (not above1 and above2) ) {
      // It crosses.  find the intersection point.
      // Simple geomety - define R = [0,1] as the proportion of
      // the length of the line traveled as you move from x1,y1
      // to x2, y2.  At some value for R, the lines intersect.
      double R = (y1-x1) / ( (x2 - x1) - (y2-y1) );
      
      // now compute the intersection point (the x and y values
      // will be the same.
      double xR = x1 + (x2-x1) * R;
      eq_points.push_back(xR);
    }
  }
  return(eq_points);
}







