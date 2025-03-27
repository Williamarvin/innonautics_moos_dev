/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleConsensus.h                               */
/*    DATE: Dec 14th, 2021                                  */
/************************************************************/

#ifndef SimpleConsensus_HEADER
#define SimpleConsensus_HEADER

#include <armadillo>
#include <cmath> // std::log
#include <algorithm>

using namespace arma;

class SimpleConsensus
{
 public:
  SimpleConsensus(double timeout);
  ~SimpleConsensus();

  // Set own name
  void setVName(std::string vname) {m_vname = vname; return;};
  void setOwnNumberOfNeighbors(unsigned int n) {m_number_of_own_neighbors = n; return;};
  
  // Get a spec for a new consensus iteration request.
  std::string requestStartConsensusSpec(std::string time);

  // Handle an incoming message with an estimate.
  bool recieveConsEstMessage(std::string vname, double time,
			     std::vector<double> estimate, unsigned int num_neighbors);

  // Handle an incoming message with a request.
  bool recieveRequestMessage(std::string msg, double msg_time, double time);

  // Get spec for a reply to a new consensus iteration request.
  std::string replyToRequestSpec();
  
  // Submit own estimate to own consensus
  bool submitOwnEstimate(std::vector<double> estimate,
		      double time);

  // Check if ready to process a new consensus iteration.
  bool readyToPerformConsensus(double time); 

  // Perform a new consensus iteration. TODO
  std::vector<double>  newConsensusItr(double time);

  
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
  // Periodicaly check if need to request a new set of consensus iterations,
  //    If needed send out a message - this can be done by
  //    any agent, it doesn't matter which - using requestConsItrSpec()
  //
  // Periodically submit own estimate using submitEstimate(estimate, variance)
  // and periodically check if a new consensus is requested (by own agent
  // or other agents.
  // If new consensus is requested, Do the following n times.
  //    First check if ready to perform iteration using
  //    readyToPerformConsensus() - (we have recieved enough messages to generate
  //    the another consensus iteration). 
  //    If ready, generate a new consensus from all messages and own
  //    estimate using newConsensusItr().
  //    Send that locally computed consensus out to all the other agents. 
  //    

  

 protected:

  // Optional function to perform distributed optimization, if
  // wanted.
  Col<double> distOptimizer(Col<double> y_i);
  
  // Member variables
  double m_timeout;
  std::string m_consensus_request_msg;
  std::string m_vname;
  unsigned int m_number_of_own_neighbors;

  double m_own_estimate_time;
  Col<double> m_own_estimate;

  // data struct to store the estimates
  // (could be combined into custom struct, TODO)
  std::map< std::string, double >  m_est_times;
  std::map< std::string, std::vector<double> > m_est_values;
  std::map< std::string, unsigned int > m_num_neighbors;
  
  
  
  
 private:

};
  


#endif 
