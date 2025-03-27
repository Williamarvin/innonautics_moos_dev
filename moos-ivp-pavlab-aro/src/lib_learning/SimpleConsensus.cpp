
/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleConsensus.cpp                             */
/*    DATE: Dec 14th, 2021                                  */
/************************************************************/

/* 
   A simple consensus algorithm for multiple agents to 
   perform distributed consensus estimation, or even distribued
   optimization.  This implementation follows that described in
   https://www.annualreviews.org/doi/10.1146/annurev-control-060117-105131
   and https://arxiv.org/abs/0711.4179. 

   The idea is that all agents using this class will be able to 
   iteratively achieve consensus about any vector value. 
   A request for a new consensus iteration can be initiated by 
   any agent using the requestConItrSpec(). 

   All agents must agree on the size of the vector to estimate, 
   and what each value in the vector represents. 

   The weights for each edge are 
   calculated using the "Metropolis weights", which is common practice, 
   and details can be found in 
   https://ieeexplore.ieee.org/abstract/document/1440896 
   or https://arxiv.org/pdf/2002.08106.pdf. 
   

*/


#include "SimpleConsensus.h"

using namespace arma;

//---------------------------------------------------------
// Constructor

SimpleConsensus::SimpleConsensus(double timeout)
{
  m_timeout = timeout;
  m_consensus_request_msg = "NEW_CONSENSUS_REQUESTED:";
  m_vname = "";
  m_own_estimate_time = 0.0;
  
  m_number_of_own_neighbors = 1000;  // Initialized large
                                     // so the first consensus
                                     // could happen and the
                                     // vector will not change much
}


//---------------------------------------------------------
// Destructor

SimpleConsensus::~SimpleConsensus()
{
}


//--------------------------------------------------------------
// Request a new consensus iteration
//    send a message like "NEW_CONSENSUS_REQUESTED:1234.5667"
std::string  SimpleConsensus::requestStartConsensusSpec(std::string time)
{
  std::string msg = m_consensus_request_msg + time;
  return(msg);
}


//--------------------------------------------------------------
// Handle an incoming message with an estimate.
bool SimpleConsensus::recieveConsEstMessage(std::string vname, double time,
					    std::vector<double> estimate, unsigned int num_neighbors)
{
  m_est_times[vname] = time;
  m_est_values[vname] = estimate;
  m_num_neighbors[vname] = num_neighbors;
  return(true);
}


//-------------------------------------------------------------
// Handle an incoming message with a request
//     Returns true if a reply is needed.
//     If the incoming message is "NEW_CONSENSUS_REQUESTED:1234.5667"
//     then msg = "NEW_CONSENSUS_REQUESTED"
//     and msg_time = 1234.5667
//     and time is the local time (now).
bool SimpleConsensus::recieveRequestMessage(std::string msg, double msg_time, double time)
{
  bool cond1 = (msg == m_consensus_request_msg);
  bool cond2 = ( (msg_time - time) < m_timeout );

  if ( cond1 && cond2 ) {
    return(true);
  } else {
    // timed out, maybe do something more here. 
    return(false);
  }  
}

//------------------------------------------------------
// Get spec for a reply to a new consensus iteration request.
//  Format is
//   vname=abe;n=3;1,2,3,4 if own estimate is present
//   where vname is unique own vehicle name, n is the number of
//   connected vehicles.  (maybe this is set by the contact manager, or
//   maybe it is the number of neighbors found on the previous iteration
//   (probably not exactly stable)
std::string SimpleConsensus::replyToRequestSpec()
{
  std::string msg = "vname=" + m_vname + ";n=";
  msg += std::to_string(m_number_of_own_neighbors);
  msg += ";";
  unsigned int length = m_own_estimate.n_elem;
  if (length == 0) {
    return(msg);
  }

  for (int i = 0; i <length; i++) {
    msg +=  std::to_string( m_own_estimate(i) );
    if ( i < (length-1) )
      msg += ",";
  }
  return(msg);
}


//----------------------------------------------------------
// Submit own estimate to own consensus
bool SimpleConsensus::submitOwnEstimate(std::vector<double> estimate, double time)
{
  m_own_estimate = conv_to< Col<double> >::from(estimate);
  m_own_estimate_time = time;
  return(true);
}

//----------------------------------------------------------
// Check if ready to perform consensus
bool SimpleConsensus::readyToPerformConsensus(double time)
{
  // Check that all estimates from own and neighboring
  // agent have not timed out.
  if ( (time - m_own_estimate_time) > m_timeout )
    return(false);

  // Check that we have at least one other estimate from the
  // neighbor that has not timed out.
  std::map<std::string, double>::iterator it;
  unsigned int count = 0;
  for (it = m_est_times.begin(); it != m_est_times.end(); it++)
    {
      if ( (time - it->second) < m_timeout )
	count++;
    }
 
  if (count == 0)
    return(false);
  
  return(true);
}


//------------------------------------------------------------
// Perform a new consensus iteration
// 
std::vector<double>  SimpleConsensus::newConsensusItr(double time)
{

  // Step 1.  Review all data we have and build vectors 
  //          to store the data for processing
  std::map<std::string, double>::iterator it;
  std::vector< Col<double> > x_neighbors_vals;
  std::vector< unsigned int > x_numb_neighbors;
  
  for (it = m_est_times.begin(); it != m_est_times.end(); it++)
    {
      // Check for timeout.
      if ( (time - it->second) < m_timeout ) {
	  //  This data is still fresh
	  //  Move into a vector of neighboring information
	  std::string vname = it->first;
	  Col<double> temp_vals = conv_to< Col<double> >::from(m_est_values[vname]);
	  x_neighbors_vals.push_back(temp_vals);
	  x_numb_neighbors.push_back(m_num_neighbors[vname]);
      }
    }
 
  // We should have already checked that we have enough fresh readings
  // using readyToPerformConsensus()
  if (x_neighbors_vals.size() == 0) {
    std::vector<double> empty_vector;
    return(empty_vector);
  }

  // Step 2.  Calculate weights for each neighbor
  // Calculate optimial weighting using 
  // https://journals.sagepub.com/doi/pdf/10.1177/1077546307077457


  // Step 3.  Calculated consensus estimate

  
  // Step 4.  (Maybe update the number of neighbors?)

  std::vector<double> dummy_return_for_now;
  return(dummy_return_for_now);
}














//----------------------------------------------------------
// Optional function to perform distributed optimization, if
// wanted.
Col<double> SimpleConsensus::distOptimizer(Col<double> y_i)
{
  //  Can implement a gradient descent scheme as described in
  // https://www.annualreviews.org/doi/10.1146/annurev-control-060117-105131
  //  TODO, for now just return the input
  return(y_i);
}
