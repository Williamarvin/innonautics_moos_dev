/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleKalmanConsensus.h                         */
/*    DATE: Dec 14th, 2021                                  */
/************************************************************/

#ifndef SimpleKalmanConsensus_HEADER
#define SimpleKalmanConsensus_HEADER

#include <armadillo>
#include <cmath> // std::log
#include <algorithm>

using namespace arma;

  // Example of how to use the consensus class
  // On each agent do the following:
  //
  // Construct the object and specify a timeout for message validity.
  //
  // Handle all incoming messages using recieveConsEstMessage(...) and
  // recieveRequestMessage(...).  If a valid request is recieved as
  // determined via recieveRequestMessage(..), then reply using
  // replyToRequestSpec()
  //
  //
  // Periodicaly check if need to request a new consensus,
  //    If needed send out a message - this can be done by
  //    any agent, it doesn't matter which - using requestStartConsensusSpec(...)
  //
  // Periodically, and when it is ready, submit new own estimate using
  // submitEstimate(estimate, variance)
  // Periodically check if a new consensus is requested (by own agent
  // or other agents using newConsensusRequested()
  //
  // If new consensus is requested, Do the following as often as required
  //    First check if ready to perform iteration using
  //    readyToPerformConsensus(...) -  
  //    If ready, generate a new consensus from all messages and own
  //    estimate using newConsensusItr(...).
  //    for running consensus, you should specify the process noise
  //    which essentially scales the weight of the previous consensus estimate
  //


class SimpleKalmanConsensus
{
 public:
  SimpleKalmanConsensus(double timeout, double wait_time);
  ~SimpleKalmanConsensus();

  // Set own name
  void setVName(std::string vname) {m_vname = vname; return;};
  void setProcessNoiseVariance(double Q_diag_var) {m_process_noise_variance = Q_diag_var; return;};
  void setKernelLengthScale(double val) {m_kernel_length_scale = val; return;};
  
  // Get a spec for a new consensus iteration request.
  void markNewConsensusRequested(double time);

  // Handle an incoming message with an estimate.
  bool receiveConsEstMessage(std::string vname, double time,
			     std::vector<double> estimate,
			     std::vector<double> variance,
			     double msg_deg_connectedness);

  // Handle an incoming message with a request.
  bool receiveRequestMessage(double msg_time, double time);

  // Overloaded functions to get spec for a reply to a new consensus
  // iteration request.  The string output is to support MOOS, and the
  // other is more general
  std::string replyToRequestSpec();
  bool replyToRequestSpec(std::string &own_vname,
			  std::vector<double> &own_estimate,
			  std::vector<double> &own_variance);
  
  // Submit own estimate to own consensus
  bool submitEstimate(double time,
		      std::vector<double> own_estimate,
		      std::vector<double> own_variance);

  // Check if ready to process a new consensus iteration.
  bool readyToPerformConsensus(double time, unsigned int max_iterations); 

  // Perform a new consensus iteration.
  bool  newConsensusItr(double time,
			std::vector< std::vector<double> > multivariate_states,
			bool use_running_consensus_when_neighbor_est_available,
			bool use_running_consensus_if_no_neighbor_est_available,
			std::vector<double> &consensus_val,
			std::vector<double> &consensus_var);

  
  bool newConsensusRequested() {return(m_new_consensus_requested);};
  double getNewConsensusRequestTime() {return(m_new_consensus_request_time);};
  

 protected:

  // calculation of minimum expected variance
  bool calcFusedMeanVariance( std::vector< Col<double> > x_vals,
			      std::vector< Mat<double> > x_covariance,
			      std::vector< double > x_deg_connectedness,
			      Col<double> &calc_val, Mat<double> &calc_covar);

  double covarianceFun(std::vector<double> v1, std::vector<double> v2);

  Mat<double> calcUnweightedCovarMarix(std::vector<std::vector<double>> multivariate_states);
  Mat<double> calcCovarMatrix(Col<double> variance);

  // Member variables

  double m_timeout;
  double m_wait_time;
  bool m_new_consensus_requested;
  bool m_first_consensus_completed;
  bool m_consensus_timed_out;
  double m_new_consensus_request_time;
  unsigned int m_consensus_iteration_number;
  bool m_previous_consensus_was_fully_connected;
  
  std::string m_vname;

  double m_own_estimate_time;
  Col<double> m_own_estimate;
  Col<double> m_own_variance;
  double m_own_connectedness;

  // data struct to store the estimates
  // (could be combined into custom struct, TODO)
  std::map< std::string, double >  m_est_times;
  std::map< std::string, std::vector<double> > m_est_values;
  std::map< std::string, std::vector<double> > m_est_variances;
  std::map< std::string, double >  m_est_deg_connectedness;
  

  // vectors for the latest consensus estimate
  Col<double> m_current_consensus_estimate;
  Col<double> m_current_consensus_variance;

  // parameters to set
  double m_kernel_length_scale = 0.005;
  double m_process_noise_variance = 0.0005;

  // Data format for "unweighted covariance matrix"
  // calculated based on the multivariate states.
  Mat<double> m_unweighted_covariance_matrix;
  
 private:

};
  


#endif 
