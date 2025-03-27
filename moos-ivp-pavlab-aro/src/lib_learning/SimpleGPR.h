/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleGPR.h                                     */
/*    DATE: June 29th, 2021                                 */
/************************************************************/

#ifndef SimpleGPR_HEADER
#define SimpleGPR_HEADER

#include <armadillo>
#include <cmath> // std::log
#include <algorithm>

using namespace arma;

class SimpleGPR
{
 public:
  SimpleGPR(double error_thresh, double variance_in_obsrvd_vals,
	    double kernel_length_scale);
  ~SimpleGPR();

  bool   recordObservation(std::vector<double> state, double val);
  
  // Basic GPR - uses the whole dataset  TODO
  bool   estimateGPR(std::vector<double> state, double &val, double &covar);

  // Generic Fast GPR  
  bool   estimateFastGPR(std::vector<double> state, double &val, double &covar);

  // Fast GPR for grids
  
  // How to do Fast GPR for grids:
  bool calcSampleSizeAndIterations();
  bool sampleAndBuild(std::vector<std::vector<double>> omit_sample_list, double omit_dist_thresh);
  bool estimateFastGPRFromSampleSet(std::vector<double> state, double &val, double &covar);

  // Example of how to use the Fast GPR for grids
  //
  // Calculate the sample size and iterations using
  // calcSampleSizeAndIterations()
  //
  // Get number of iterations
  // getFastGPRIterations()
  // 
  //
  // Each iteration do
  //    Sample the observations and build the covariance matrix
  //    using sampleAndBuild()
  //
  //    For each x,y grid location do:
  //       Caluculate estimate at x, y using
  //       estimateFastGPRFromSampleSet()
  //
  //       Update running average
  
  unsigned int  getFastGPRIterations() const { return(m_fastGPR_iterations);}
  

 protected:
  double covarianceFun(const std::vector<double> &v1, const std::vector<double> &v2);  
  double covarianceFun(const double &dist);  // if you already have the distance between two states

  long long unsigned int  m_fastGPR_sample_size;
  unsigned int  m_fastGPR_iterations;
  
  double m_error_thresh;
  double m_variance_in_observed_values;
  double m_kernel_length_scale;


  // Holds the states for each recorded observation.
  // Each row (outer index) is an individual observation,
  // each column is a different state (x, y, etc).
  std::vector< std::vector<double> >  m_observed_states;  

  // Holds the value of each observation.
  // Entries in each row corresponds to the 
  // same rows in the observed states.
  std::vector<double>  m_observed_values;

  Mat<double> m_K;          // Covariance matrix
  Mat<double> m_L_cholesky; // Matrix to save the Cholesky decomposition
                            // of matrix K
  Mat<double> m_alpha;      // Alpha matrix to save computation
  

  Col<double> m_y;   // output or observed values (y = f(x) + noise).

  std::vector<int> m_sampled_index;  // vector of indexes randomly choosen
                                   // to generate the estimate.
  
 private:

};
  


#endif 
