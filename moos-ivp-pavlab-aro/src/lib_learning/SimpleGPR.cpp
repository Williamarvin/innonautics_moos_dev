
/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleGPR.cpp                                   */
/*    DATE: June 29th, 2021                                 */
/************************************************************/

/* 
   A simple Gaussian process regression (GPR) class for
   gaussian interpolation.  This class estimates the mean and 
   covariance of the scalar value that corresponds to a given
   set of input variables using linear unbiased prediction.  
   This class implements a Fast GPR as described in this paper
   https://arxiv.org/abs/1509.05142. It also uses the Cholesky 
   decomposition to save computational time as described in 
   https://gregorygundersen.com/blog/2019/09/12/practical-gp-regression/
   
   The covariance function is the squared L2 norm exponential. 
   Change it by editing the covarianceFun(...)

*/



#include "SimpleGPR.h"

using namespace arma;

//---------------------------------------------------------
// Constructor

SimpleGPR::SimpleGPR(double error_thresh, double variance_in_obsrvd_vals,
		     double kernel_length_scale)
{
  m_error_thresh = error_thresh;
  m_variance_in_observed_values = variance_in_obsrvd_vals;
  m_kernel_length_scale = kernel_length_scale;
  m_fastGPR_iterations = 1;
  m_fastGPR_sample_size = 1;
}


//---------------------------------------------------------
// Destructor

SimpleGPR::~SimpleGPR()
{
}


//--------------------------------------------------------------
// Record Observations
bool  SimpleGPR::recordObservation(std::vector<double> state, double val)
{
  m_observed_states.push_back(state);
  m_observed_values.push_back(val);
  return true;
}


//---------------------------------------------------------
// Calculate Sample Size and Iterations
//  Taken from https://arxiv.org/abs/1509.05142.
bool SimpleGPR::calcSampleSizeAndIterations()
{
  unsigned int N = m_observed_states.size();

  // Not enough observations to even try
  if (N <=5)
    return false;

  if (N <=20) {
    // Not enough observations to sample,
    // just use the full set and one iteration
    m_fastGPR_sample_size = m_observed_states.size();
    m_fastGPR_iterations = 1;
    return true;
  }
  
  double N_dbl = (double) N;

  // Best sample size found in the paper is
  // N ^ (1/log(log(N))) / (error_thresh)^0.1
  double delta = 1.0 / (std::log ( std::log (N_dbl)));

  double N_s_dbl = std::floor( 5.0 * pow(N_dbl, delta) / pow(m_error_thresh, 0.1) );
  m_fastGPR_sample_size = (int) N_s_dbl;

  // Clamp the sample size to the total number of observed states
  if (m_fastGPR_sample_size > m_observed_states.size() )
    m_fastGPR_sample_size = m_observed_states.size();
  

  // According to the paper K = 30 is a good number
  // of estimators. We find K = 25 to be good to
  m_fastGPR_iterations = 25;

  return true;
}

//---------------------------------------------------------
// Calculate Sample Size and Iterations
//  Input:  omit_sample_list = vector of vectors for locations to not
//                             to sample around
//          omit_dist_thresh = the "distance" around each location in the
//                             omit_sample_list where only one sample can
//                             selected.
//                             Ex. if {1,2} is on the omit_sample_list and
//                             omit_dist_thresh = 1.0, then only one sample
//                             may be selected in a circle of radius 1 centered
//                             at x=1, y=2.  
//        
bool SimpleGPR::sampleAndBuild(std::vector<std::vector<double>> omit_sample_list, double omit_dist_thresh)
{

  // Step 1. Sample
  m_sampled_index.clear();
  // initialize random seed
  unsigned long int tseed = time(NULL);
  unsigned long int pseed = getpid() + 1;
  unsigned int rseed = (tseed*pseed) % 50000;
  srand(rseed);

  // Generate list of valid random samples 
  int index;  // Variable for random index
  unsigned int rejected_count = 0; // count the number of times we
                                   // have tried unsucessfully to
                                   // find a valid sample

  // Generate a vector to ensure that we only use one sample near the
  // locations in the omit list.
  std::vector<bool> alreadly_sampled_one_on_omitted_list( omit_sample_list.size(), false);

  // Continue sampling until we have reached the number of samples requested
  // or until we have been rejected more times than the number of possible 
  // samples remaining.  This is conservative, since this is a random process
  // and we could have more than one rejection for a given sample.
  // The second condition effectively enforces a conservative bound on the
  // while loop. 
  while( (m_sampled_index.size() < m_fastGPR_sample_size) 
	 and (rejected_count < (m_observed_values.size() - m_sampled_index.size() - omit_sample_list.size() ) ) ) {
    // Randomly sample
    index = (std::rand() % m_observed_values.size());
    bool ok_to_add = true;
     
    // Check if we already have this index
    std::vector<int>::iterator it;
    it = std::find(m_sampled_index.begin(), m_sampled_index.end(), index);
    if (it == m_sampled_index.end() ) {

      // Check if sample is "close" to one on the omission list
      // Closeness is defined per the covariance function.
      // Only allow one index to be added that is close to the location
      // in the admit list. 
      for (unsigned int i = 0; i < omit_sample_list.size(); i++)
	{
  
	  double val2;
	  val2 = covarianceFun(m_observed_states[index], omit_sample_list[i]);

	  // compute the threshold using the same covariance function
	  double thresh = covarianceFun( omit_dist_thresh*omit_dist_thresh ); 
	  if (val2 > thresh){
	    // this random sample is close to one on the omit list
	    // do not add it if we already have one in this area
	    if (not alreadly_sampled_one_on_omitted_list[i]) {
	      // we will add this one and set the flag
	      // leave the ok_to_add  flag alone
	      alreadly_sampled_one_on_omitted_list[i] = true;
	    } else {
	      ok_to_add = false;
	    }
	    // We have found one entry on the omit list that is
	    // close and have either marked this to add, or that
	    // we need to pick another random sample. 
	    break;
	  }
	} // end of for loop

      // Check that not in list and not close to any in the
      // omit list
      if (ok_to_add) {
	m_sampled_index.push_back(index);
	rejected_count = 0;
      } else {
	rejected_count ++;
      }
    } // end of if not alreay in list 
  } // end of while loop

  // Finally, reset
  // Reset the sample size to be the number actually achieved
  m_fastGPR_sample_size = m_sampled_index.size();
  
  // Step 2.  Build
  // Build the covariance matrix and y vector
  // Create a matrix of samples
  // initialize
  uword sample_size = m_fastGPR_sample_size; // "convert" from unsigned int to uword
  m_K = zeros(sample_size, sample_size);
  m_y = zeros(sample_size);

  // Covariance matrix is symmetric
  double covar;
  for (int k=0; k<m_sampled_index.size(); k++){
    for (int p=k; p<m_sampled_index.size(); p++){
      std::vector<double> v_k = m_observed_states[ m_sampled_index[k] ];
      std::vector<double> v_p = m_observed_states[ m_sampled_index[p] ];
      covar = covarianceFun(v_k, v_p);

      if (k == p) {
	m_K(k,p) = covar + m_variance_in_observed_values;	
      } else {
	m_K(k,p) = covar;
	m_K(p,k) = covar;
      }
      
    }
  }

  // Calculate the cholesky decomp of the covariance matrix once
  // and save it for speed.
  m_L_cholesky = arma::chol(m_K, "lower");       // Need to specify  a
                                                 // lower triangular matrix

  // Build the y matrix
  for (int j=0; j<m_sampled_index.size(); j++){
    m_y(j) = m_observed_values[ m_sampled_index[j] ];
  }

  // Calculate the alpha matrix
  // Specifically indicate the matrices are triangular for speed.
  // See arama documentation for more info. 
  m_alpha = arma::solve( arma::trimatu( arma::trans(m_L_cholesky) ), arma::solve( arma::trimatl(m_L_cholesky), m_y) );

  return true;
}

//----------------------------------------------------
// Procedure: covarianceFunction
//            Calculated the covariance as the squared
//            exponential.  Change if desired
double SimpleGPR::covarianceFun(const std::vector<double> &v1, const std::vector<double> &v2)
{
  if (v1.size() != v2.size())
    return(0.0);
  
  double val = 0.0;
  for (int i=0; i<v1.size(); i++){
    val = std::pow( (v1[i] - v2[i]), 2) + val;
  }
  
  return( covarianceFun(val) );

}


//----------------------------------------------------
// Procedure: covarianceFunction
//            Calculated the covariance as the squared
//            exponential.
//            THE VALUE DIST IS ASSUMED TO BE SQUARED!
//            
double SimpleGPR::covarianceFun(const double &dist)
{
  return(std::exp(-m_kernel_length_scale*dist));
}

//------------------------------------------------------
// Procedure: estimateFastGPRFromSampleSet
bool SimpleGPR::estimateFastGPRFromSampleSet(std::vector<double> state, double &val, double &covar)
{

  // Build K_star
  uword sample_size = m_fastGPR_sample_size; // "convert" from unsigned int to uword
  Col<double> m_K_star(sample_size, fill::zeros);
  
  for (int j=0; j<m_fastGPR_sample_size; j++){
    std::vector<double> v_j = m_observed_states[ m_sampled_index[j] ];
    m_K_star(j) = covarianceFun(v_j, state);
  }

  val = arma::dot( m_K_star, m_alpha);

  Col<double> v;
  v = arma::solve( arma::trimatl(m_L_cholesky), m_K_star);
  covar = covarianceFun(state, state) - 1.0 * arma::dot(v, v);
  
  return(true);
}

//------------------------------------------------------
// Procedure: estimateFastGPRFromSampleSet
//            Useful when only one estimate is needed
//            instead of a repeated type estimate such as
//            a grid.
bool SimpleGPR::estimateFastGPR(std::vector<double> state, double &val, double &covar)
{
  
  // Calculate the sample size and iterations using
  calcSampleSizeAndIterations();

  double running_average_val = 0.0;
  double running_average_covar = 0.0;

  double jth_val;
  double jth_covar;

  for (int j=1; j<=m_fastGPR_iterations; j++) {
    std::vector<std::vector<double>> empty_omit_list;
    sampleAndBuild(empty_omit_list, 0.0);
    estimateFastGPRFromSampleSet(state, jth_val, jth_covar);
    
    double j_dbl = (double) j;
    running_average_val = (jth_val + running_average_val * (j_dbl-1.0) ) / j_dbl;
    running_average_covar = (running_average_covar*pow( (j_dbl-1.0),2)+jth_covar)/(pow((j_dbl),2));
  }

  val = running_average_val;
  covar = running_average_covar;

  return(true);
}

//-----------------------------------------------------
// Proceedure:  estimateGPR
//              Basic gaussian process regression,
//              no sampling.
//              Be careful! This might consume A LOT of
//              memory, the fast version exists for a reason

bool SimpleGPR::estimateGPR(std::vector<double> state, double &val, double &covar)
{
  // Clear the matricies
  uword sample_size = m_observed_values.size(); // "convert" from unsigned int to uword
  m_K = zeros(sample_size, sample_size);
  m_y = zeros(sample_size);
  
  // Build the covariance matrix and y vector
  // Covariance matrix is symmetric
  double covar_kp;
  for (int k=0; k<m_observed_values.size(); k++){
    for (int p=0; p<m_observed_values.size(); p++){
      std::vector<double> v_k = m_observed_states[k];
      std::vector<double> v_p = m_observed_states[p];
      covar_kp = covarianceFun(v_k, v_p);
      m_K(k,p) = covar_kp;
      m_K(p,k) = covar_kp;
    }
  }

  // Add in the variance of the obs vals to m_K
  m_K = m_K + arma::eye(sample_size, sample_size) * m_variance_in_observed_values;

  // Calculate the inverse of the covariance matrix once
  // and save it for speed.
  m_L_cholesky = arma::chol(m_K, "lower");       // Need to specify  a
                                                 // lower triangular matrix

  // Build the y matrix
  for (int j=0; j<m_observed_values.size(); j++){
    m_y(j) = m_observed_values[j];
  }

  // Calculate the alpha matrix
  // Specifically indicate the matrices are triangular for speed.
  // See arama documentation for more info. 
  m_alpha = arma::solve( arma::trimatu( arma::trans(m_L_cholesky) ), arma::solve( arma::trimatl(m_L_cholesky), m_y) );
    

  // Complete the estimation
  estimateFastGPRFromSampleSet(state, val, covar);
  
  return(true);
}
