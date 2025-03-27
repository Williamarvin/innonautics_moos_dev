
/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleKalmanConsensus.cpp                       */
/*    DATE: Dec 14th, 2021                                  */
/************************************************************/

/* 
   A simple consensus algorithm for combining multiple local
   Gaussian process regressions using Kalman consensus.

   The idea is that all agents using this class will be able to 
   iteratively achieve consensus about any vector value. 
   A request for a new consensus iteration can be initiated by 
   any agent using the requestConItrSpec(). 

   All agents must agree on the size of the vector to estimate, 
   and what each value in the vector represents. 
 
   The weights for each edge are calculated to minimuze 
   the expected variance of the fused result.  This derivation 
   can be found in 
   https://ieeexplore.ieee.org/document/1470240
   and the improved unbiased Kalman Consensus is implemented 
   as described in 
   https://ieeexplore.ieee.org/document/1657263


*/


#include "SimpleKalmanConsensus.h"

using namespace arma;

//---------------------------------------------------------
// Constructor

SimpleKalmanConsensus::SimpleKalmanConsensus(double timeout, double wait_time)
{
  m_timeout = timeout;
  m_wait_time = wait_time;
  m_vname = "";
  m_own_estimate_time = 0.0;
  m_new_consensus_request_time = 0.0;
  m_consensus_iteration_number = 0;
  //m_previous_consensus_was_fully_connected = false;
  m_new_consensus_requested = false;
  m_first_consensus_completed = false;
  m_consensus_timed_out = false;
  m_own_connectedness = 1.0;
  m_process_noise_variance = 0.0005;
  
}


//---------------------------------------------------------
// Destructor

SimpleKalmanConsensus::~SimpleKalmanConsensus()
{
}


//--------------------------------------------------------------
// Mark that a new consensus was requested 
void  SimpleKalmanConsensus::markNewConsensusRequested(double time)
{
  m_new_consensus_requested = true;
  m_new_consensus_request_time = time;
  m_consensus_iteration_number = 0;
  return;
}


//--------------------------------------------------------------
// Handle an incoming message with an estimate.
//       Incoming message is from vehicle with "vname",
//       time = local time the message was recieved
//       etimate = vector of (mean) values in the estimate
//       variance = vector of variance associated with each
//                  expected value
bool SimpleKalmanConsensus::receiveConsEstMessage(std::string vname, double time,
						  std::vector<double> estimate,
						  std::vector<double> variance,
						  double msg_deg_connectedness)
{
  m_est_times[vname] = time;
  m_est_values[vname] = estimate;
  m_est_variances[vname] = variance;
  m_est_deg_connectedness[vname] = msg_deg_connectedness;
  return(true);
}


//-------------------------------------------------------------
// Handle an incoming message with a request
//     Returns true if a reply is needed.
//     and double val = "1234.5667"
//     then msg_time = 1234.5667
//     and time is the local time (now).
bool SimpleKalmanConsensus::receiveRequestMessage(double msg_time, double time)
{
  if ( (msg_time - time) < m_timeout ) {
    m_new_consensus_requested = true;
    m_new_consensus_request_time = time;
    m_consensus_iteration_number = 0;
    return(true);
  } else {
    // timed out, maybe do something more here;
    return(false);
  }  
}

//------------------------------------------------------
// Get spec for a reply to a new consensus iteration request.
//  The is a helper function for MOOS
//  Format is
//   vname=abe;;; if no own estimate and variance is present
//   vname=abe;1,2,3,4;5,6,7,8;2 if own estimate, variance,
//   and connectedness is present
std::string SimpleKalmanConsensus::replyToRequestSpec()
{
  std::string msg = "vname=" + m_vname + ";";

  unsigned int length = m_own_estimate.n_elem;
  unsigned int length2 = m_own_variance.n_elem;

  if (length == 0) {
    msg += ";;";
    return(msg);
  }

  for (unsigned int i = 0; i <length; i++) {
    if (m_consensus_iteration_number == 0)
      msg +=  std::to_string( m_own_estimate(i) );
    else
      msg +=  std::to_string( m_current_consensus_estimate(i) );
    
    if ( i < (length-1) )
      msg += ",";
  }
  
  msg += ";";
  if (length2 == 0){
    msg += ";";
    return(msg);
  }

  for (unsigned int j = 0; j <length2; j++) {
    if (m_consensus_iteration_number == 0)
      msg +=  std::to_string( m_own_variance(j) );
    else
      msg +=  std::to_string( m_current_consensus_variance(j) );
    
    if ( j < (length-1) )
      msg += ",";
  }

  msg += ";";
  if (m_own_connectedness != 0.0) {
    msg += std::to_string( m_own_connectedness);
  }
  
  return(msg);
}


//---------------------------------------------------------
// Get spec without string
//      returns false if error
bool SimpleKalmanConsensus::replyToRequestSpec(std::string &own_vname,
					       std::vector<double> &own_estimate,
					       std::vector<double> &own_variance)
{
  if (m_vname == "")
    return(false);
  if (m_own_estimate.n_elem == 0)
    return(false);
  if (m_own_variance.n_elem == 0)
    return(false);

  own_vname = m_vname;
  own_estimate = conv_to< std::vector<double> >::from(m_own_estimate);
  own_variance = conv_to< std::vector<double> >::from(m_own_variance);

  return(true);
}



//----------------------------------------------------------
// Submit own estimate to own consensus
bool SimpleKalmanConsensus::submitEstimate(double time, std::vector<double> own_estimate, std::vector<double> own_variance)
{
  m_own_estimate = conv_to< Col<double> >::from(own_estimate);
  m_own_estimate_time = time;
  m_own_variance = conv_to< Col<double> >::from(own_variance);
  return(true);
}

//----------------------------------------------------------
// Check if ready to perform consensus
//         This proceedure runs through a set of checks and
//         takes action accordingly. 
bool SimpleKalmanConsensus::readyToPerformConsensus(double time, unsigned int max_iterations)
{
  // Check 0:
  // First check that a consensus was requested
  // This might be a redundant check, but just in case
  if (!m_new_consensus_requested) {
    return(false);
  }

  // Check 1:
  // Then check if we have already completed the maximum allowed
  // iterations
  bool cond1 = (m_consensus_iteration_number >= max_iterations);
  if ( cond1 ) {
    // we are done with this request
    m_new_consensus_requested = false;
    m_consensus_timed_out = false;
    // reset the counters
    m_consensus_iteration_number = 0;
    m_own_connectedness = 1.0;
    return(false);
  }

  // Check 2:
  // Then check we have waited long enough for messages
  // to come in.
  double time_to_wait_accounting_for_iterations;
  time_to_wait_accounting_for_iterations = m_wait_time * ( static_cast<double>(m_consensus_iteration_number + 1) );
  if ( (time - m_new_consensus_request_time) < time_to_wait_accounting_for_iterations ) {
    return(false);
  }


  // Check 3:
  // Check that estimates from own agent exists.
  // The assumption is replies from
  // other agents may come much quicker than the GPR
  // processing own data can finish. 

  if ( (m_own_estimate.n_elem == 0) and (m_own_variance.n_elem == 0) ) {
    return(false);
  }


  // Pre Check 4 and 5:
  // Calculate the number of valid messages from neighbors
  // where valid = has not timed out
  // if this is after the first iteration, check if all
  // neighbors are reporting equal connectedness.  The
  // connectedness check must be done after at least one iteration
  // because the connectedness reported on each message is the
  // connectedness from the previous iteration.


  unsigned int count = 0;
  unsigned int old_msg = 0;
  //m_previous_consensus_was_fully_connected = true;
  std::map<std::string, double>::iterator it;
  
  for (it = m_est_times.begin(); it != m_est_times.end(); it++){
    // count as appropriate
    if ( (time - it->second) < m_timeout ) {
      count++;
      
      // this hasn't timed out
      // check if connectedness is equal
      //std::string vname = it->first;
      //if ( m_est_deg_connectedness[vname] != m_own_connectedness) {
      //m_previous_consensus_was_fully_connected = false;
      //}
      
    } else {
      old_msg++;
    }
  }

  cout << "Number of valid Neighbors found = " << count << " and invalid found = " << old_msg << endl;

  // Check 4:
  // Then check if we had done one iteration, and
  // all vehicles are reporting the same connectedness number.
  // This means that we had a full spanning tree last time
  // and we should stop
  /*
  if ( m_previous_consensus_was_fully_connected and (m_consensus_iteration_number > 0) ) {
    // we are done with this request
    m_new_consensus_requested = false;
    m_consensus_timed_out = false;
    // reset the counters
    m_consensus_iteration_number = 0;
    m_own_connectedness = 1.0;
    return(false);
  }
  */

  // Check 5:
  // Check that we have at least one other estimate from the
  // neighbor that has not timed out.
  // and that this is the first consensus.
  // If this is the first consensus, then returning true is ok
  // after checking for timeout.
  
  // If we do not have any fresh data from neighbors, AND we
  // have completed at least one consensus, then we are not ready.
  // Unless we have exceeded the timeout, then set the flag
  // to insure the "final" consensus actions are completed.
  if ( (count == 0) and (m_first_consensus_completed) ) {
    
    // Now check if we have exceeded our time limit
    // This might be due to no connections to other agents
    // at any time, OR intermittent connection that stops
    // part of the way throught the consensus and we have
    // no more fresh messages
    double max_time_to_wait_accounting_for_iterations;
    max_time_to_wait_accounting_for_iterations = m_wait_time * ( static_cast<double>(max_iterations) );
    bool cond1 = ( (time - m_new_consensus_request_time) < max_time_to_wait_accounting_for_iterations );

    if (cond1) {
      cout << "False:  No data from neigbors and not first consensus and we have not timed out" << endl;
      return(false);
    } else {
      // We are ready to perform our "final" consensus for this
      // round that has timed out. Set the flag and continue on
      m_consensus_timed_out = true; 
    }
  }



  return(true);
}


//------------------------------------------------------------
// Perform a new consensus iteration
//                   time = local time now
//    multivariate_states = used for the probability density function
//                          for the covariance matrix
//use_running_consensus.. = true = include last consensus if available.
//                          when estimates from other neighbors are available
// use_running_cons_if .. = true use running consensus if we have timed out
//                        = and no other estimates from neighbors are available
//
//    Updated by reference:
//          consensus_val = estimate that minimizes the covariance
//          consensus_var = variance of the estimate. 
// 
//    returns false if error
bool  SimpleKalmanConsensus::newConsensusItr(double time,
					     std::vector< std::vector<double> > multivariate_states,
					     bool use_running_consensus_when_neighbor_est_available,
					     bool use_running_consensus_if_no_neighbor_est_available,
					     std::vector<double> &consensus_val,
					     std::vector<double> &consensus_var)
{
  // Step 0.  Initial Bookkeeping
  //          If this is the first consensus request, and we don't have any
  //          data from neighbors,
  //          then just pass the own estimate and variance
  //          so the path planner has something to work with.
  
  bool cond1 = (m_est_times.size() == 0) and (not m_first_consensus_completed);
  if ( cond1 ) {

    // own estimate should exist because we checked that it hasn't timed out.
    consensus_val = conv_to< std::vector<double> >::from(m_own_estimate);
    consensus_var = conv_to< std::vector<double> >::from(m_own_variance);

    // update the current consensus variables for completeness
    m_current_consensus_estimate = m_own_estimate;
    m_current_consensus_variance = m_own_variance;

    // Set flags
    m_first_consensus_completed = true;
    return(true);

  }

  // Step 0.1 Initial Bookkeeping cont,
  //            If the consensus has timed out, and we don't want to do a
  //            running consensus to finish, then return the best estimate
  //            we have now,  and clean up.
  if (m_consensus_timed_out and not use_running_consensus_if_no_neighbor_est_available) {
    // We have run out of time to do the consensus.
    // If we have an incomplete consensus, then return what we have so far
    // If we never even started, then return own estimate
    if (m_consensus_iteration_number > 0) {
      consensus_val = conv_to< std::vector<double> >::from(m_current_consensus_estimate);
      consensus_var = conv_to< std::vector<double> >::from(m_current_consensus_variance);
      // leave the m_own_connectedness as it was
    } else {
      // own estimate should exist because we checked that it hasn't timed out.
      consensus_val = conv_to< std::vector<double> >::from(m_own_estimate);
      consensus_var = conv_to< std::vector<double> >::from(m_own_variance);
      m_own_connectedness = 1.0;
    }
    // either way we are done with this request
    m_new_consensus_requested = false;
    m_consensus_timed_out = false;
    // reset the counters
    m_consensus_iteration_number = 0;
    // but keep the fully connected flag the way it is. 
    
    return(true);
  }

  // Step 1.  Review all data we have and build vectors (of Vecs)
  //          as requested to store the data for processing
  std::vector< Col<double> > x_vals;
  std::vector< Mat<double> > x_covariance;
  std::vector< double > x_deg_connectedness;

  // calculate the "unweighted covariance matrix"
  // based on the multivariate states
  m_unweighted_covariance_matrix = calcUnweightedCovarMarix(multivariate_states);

  // First add own estimates and variance.
  if (m_consensus_iteration_number < 1 ) {
    x_vals.push_back(m_own_estimate);
    x_covariance.push_back( calcCovarMatrix(m_own_variance) );
  } else {
    x_vals.push_back(m_current_consensus_estimate);
    x_covariance.push_back( calcCovarMatrix( m_current_consensus_variance) );
  }
 
  // Next add all neighbors that have not timed out.
  std::map<std::string, double>::iterator it;
  for (it = m_est_times.begin(); it != m_est_times.end(); it++)
    {
      // Check for timeout.
      if ( (time - it->second) < m_timeout ) {
	  //  This data is still fresh
	  //  Move into a vector of neighboring information
	  std::string vname = it->first;
	  Col<double> temp_vals = conv_to< Col<double> >::from(m_est_values[vname]);
	  x_vals.push_back(temp_vals);
	  
	  Col<double> temp_vars = conv_to< Col<double> >::from(m_est_variances[vname]);
	  x_covariance.push_back( calcCovarMatrix(temp_vars) );

	  x_deg_connectedness.push_back(m_est_deg_connectedness[vname]);
      }
    }

  // For simplicity/readability determine if we are in a special end condition, i.e.
  // if we have timed out, and we have requested to fuse the previous consensus
  // with the current own estimate for the final output
  bool in_special_end_condition = (m_consensus_timed_out and use_running_consensus_if_no_neighbor_est_available);

    

  // Now check that we have variances for all estimates. 
  // We should have already checked that we have enough fresh readings
  // using readyToPerformConsensus(), but just in case.
  //if ( (x_vals.size() == 1) or (x_covariance.size() == 1) ) {
  if ( ( (x_vals.size() == 1) or (x_covariance.size() == 1) ) and not in_special_end_condition ){
    cout << "  Con: Error: no data from neighbors" << endl;
    return(false);
  }
  // otherwise, update the own degree of connectedness
  // need to update the first entry in the vector, there might be a better
  // way to do this, but it needs to happen after the other vectors are
  // assembled. 
  m_own_connectedness = (double) (x_vals.size()-1);
  // Bound own connectedness greater than 0.0 otherwise we will be dividing by zero.
  // This also prevents the own_connectedness from becoming 0 if in
  // "special end condition"
  if (m_own_connectedness < .0001) {m_own_connectedness = 1.0;};
  x_deg_connectedness.insert(x_deg_connectedness.begin(), m_own_connectedness);
  

  // Finally, include the last consensus estimate if available and requested
  //    Two conditions are possible:
  bool cond3 = (m_first_consensus_completed
		and  use_running_consensus_when_neighbor_est_available
		and (m_consensus_iteration_number < 1 ));
  bool cond4 = (m_first_consensus_completed and in_special_end_condition);
  if (cond3 or cond4) {
    x_vals.push_back(m_current_consensus_estimate);

    // update the covariance matrix to include process noise.
    Mat<double> posterior_covar = calcCovarMatrix(m_current_consensus_variance);

    unsigned int length = posterior_covar.n_rows;
    Mat<double> Q_process_noise(length, length, arma::fill::eye);
    
    posterior_covar = posterior_covar + Q_process_noise * m_process_noise_variance;
    x_covariance.push_back( posterior_covar );
    x_deg_connectedness.push_back(1.0);
  }


  // Step 2.  Calculate the new consensus using optimial weighting via
  // https://ieeexplore.ieee.org/document/1470240
  // 
  // First entry in the vectors is own estimate

  // initialize intermediate matrix to return 
  Mat<double> new_consensus_covar;

  // Update the estimate using Kalman's approach
  bool ok;
  ok = calcFusedMeanVariance(x_vals, x_covariance, x_deg_connectedness,
			     m_current_consensus_estimate, new_consensus_covar);
  if (not ok) {
    return(false);
    cout << "   Con: Error: Kalman calc failed." << endl;
  }
  m_current_consensus_variance = new_consensus_covar.diag();


  // return new consensus values.
  consensus_val = conv_to< std::vector<double> >::from(m_current_consensus_estimate);
  consensus_var = conv_to< std::vector<double> >::from(m_current_consensus_variance);

  // Finally, update some flags
  m_first_consensus_completed = true;
  m_consensus_iteration_number++;
 
  
  return(true);
}


//--------------------------------------------
// Calculation of minimum expected variance
//     Performs safety checking to prevent divide by
//     zero and returns false if error,
//     First entry in the vector is own estimate
bool SimpleKalmanConsensus::calcFusedMeanVariance( std::vector< Col<double> > x_vals,
						   std::vector< Mat<double> > x_covariance,
						   std::vector<double> x_connectedness,
						   Col<double> &calc_val, Mat<double> &calc_covar)
{
   // get length of Column 
  unsigned int length = x_vals[0].n_elem;
  unsigned int n_agents = x_vals.size();

  Mat<double> sum1(length, length, arma::fill::zeros);
  Col<double> sum2(length, arma::fill::zeros);

  
  // Calculate Sums
  // first entry in the vectors is own estimate
  for (unsigned int j=1; j < n_agents; j++) {
    
    Mat<double> temp;
    double connectedness_scale_factor = x_connectedness[j];
    if (connectedness_scale_factor < 1.0){
      connectedness_scale_factor = 1.0;
      cout << "  Warning in Kalman Update:  Had to reset scale factor to 1 " << endl;
      cout << "  Check that messages have connectedness numbers of at least 1 " << endl;
    }
      
    bool ok1 = arma::inv_sympd(temp, connectedness_scale_factor * x_covariance[j]);

    if (not ok1) {
      cout << "   Con error:  The j=" << j << " covariance matrix has a bad condition number="
	   << arma::cond( x_covariance[j])<< endl;
      return(false);
    }
    sum1 = temp + sum1;
    sum2 = temp * ( x_vals[j] - x_vals[0] ) + sum2;   
  }

  // calculate new estimate and variance
  // calculate the variance
  Mat<double> covar_invs;
  bool ok2 = arma::inv_sympd(covar_invs, x_covariance[0]);

  if (not ok2) {
   cout << "   Con error:  The own_covar matrix has a bad condition number = "
	<< arma::cond( x_covariance[0] ) << endl;
    return(false); 
  } else {
    // add the sum
    covar_invs = covar_invs + sum1;
  }

  Mat<double> new_calc_covar;
  bool ok3 = arma::inv_sympd(new_calc_covar, covar_invs);

  if (not ok3) {
    cout << "   Con error:  The covariance inverse matrix has a bad condition number = "
	 << arma::cond( covar_invs ) << endl;
    return(false);
  }

  calc_covar = new_calc_covar;

  // calculate estimate
  calc_val = x_vals[0] + (new_calc_covar * sum2);
  
  return(true);
}



//----------------------------------------------------
// Procedure: covarianceFunction
//            Calculated the covariance as the squared
//            exponential.  Change if desired
double SimpleKalmanConsensus::covarianceFun(std::vector<double> v1, std::vector<double> v2)
{
  if (v1.size() != v2.size())
    return(0.0);
  
  double val = 0.0;
  for (int i=0; i<v1.size(); i++){
    val = std::pow( (v1[i] - v2[i]), 2) + val;
  }
  return(std::exp(-m_kernel_length_scale*val));

}


//----------------------------------------------------
// Procedure: calculate the "unweighted covariance matrix"
//            basically a multivariate gaussian without the
//            variance known
Mat<double> SimpleKalmanConsensus::calcUnweightedCovarMarix(std::vector<std::vector<double>> multivariate_states)
{
  unsigned int length = multivariate_states.size();
  Mat<double> mat(length, length, arma::fill::eye);

  // calcluate the off diagonals. 
  for ( unsigned int i=0; i<length; ++i)
    {
      for ( unsigned int j=i; j<length; ++j)
	{
	  double val = covarianceFun(multivariate_states[i], multivariate_states[j]);
	  mat(i,j) = val;
	  mat(j,i) = val;	  
	}
    }

  return(mat);
}


//-------------------------------------------------------
// Procedure:  calc Covar Matrix
//             using the unweighted covarince matrix and
//             the vector of variances
Mat<double> SimpleKalmanConsensus::calcCovarMatrix(Col<double> variance)
{
  // calculate the "unweighted covariance matrix"
  // based on the multivariate states
  unsigned int length = variance.n_elem;
  Mat<double> mat(length, length, arma::fill::zeros);
  // calcluate the entries in the symetric matrix diagonals. 
  for ( unsigned int i=0; i<length; ++i)
    {
      for ( unsigned int j=i; j<length; ++j)
	{
	  double val = m_unweighted_covariance_matrix(i,j) * std::sqrt(variance[i]) * std::sqrt(variance[j]);
	  mat(i,j) = val;
	  mat(j,i) = val;
	}
    }

  return(mat);
}
