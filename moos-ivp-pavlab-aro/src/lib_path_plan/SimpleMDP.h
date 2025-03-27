/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleMDP.cpp                                   */
/*    DATE: March 7th, 2022                                 */
/************************************************************/

#ifndef SIMPLEMDP_HEADER
#define SIMPLEMDP_HEADER
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <numeric>  // inner product, accumulate;
#include <algorithm> // find
#include <cmath>   // erf, log, pow, PI



class SimpleMDP
{
 public:
  SimpleMDP();
  ~SimpleMDP();

    
  // Value Iteration (Figure 17.4) in AI: A Modern Approach
  //                Inputs:   rewards = a map of states and reward values
  //                                    ex rewards["node1"] = 10. It is
  //                                    assumed that all the nodes are included,
  //                                    even those that have no reward (=0)
  //                          edges   = a vector of directed edges between nodes.
  //                                    These are treated like actions in
  //                                    the value iteration.
  //                                    edges[n] = {"node1","node2"} documents
  //                                    it is possible to move from node1 to node2
  //                         discount = a value in the range (0,1) to discount
  //                                    the reward of the next state
  //                          epsilon = the maximum error allowed in the utility
  //                                    of any state
  //                         max_iter = maximum iterations allowed for value
  //                                    iteration, default = 50
  //
  //                Outputs:  Utility = a map of states and estimated utility
  //                                    ex U["node1"] = 13.1;
  //
  //                The transition model P(s'|s,a) must be defined before using!!
  //    
  std::map<std::string, double> valueIteration(const std::map<std::string,double>& rewards,
					       const std::vector<std::pair<std::string,std::string> >& edges,
					       const double& discount,
					       const double& epsilon,
					       const int max_iter);


  // stateTransistionProbability
  //                 Must be customized for every unique model format, but the inputs and outputs
  //                 should be the same.
  //                 Inputs:         possible_next_state = a vector of possible next states
  //                                                       ex {"node2", "node3", "node4"}
  //                                       current_state = the current state ex "node1"
  //                                               index = index of the current action
  //                                                       under consideration.  ex index = 1
  //                                                       would indicate that an action to
  //                                                       move to "node3" in the above example
  //
  //                 Outputs:    transistion probability = a vector of probabilites for each state,
  //                                                       given the current state and intended action
  //                                                       ex { 0.1, 0.8, 0.1} could be an distibution
  //                                                       of probabilities for the example above
  //                                                       where prob("node2") = 0.1,
  //                                                       prob("node3") = 0.8, prob("node4") = 0.1
  //                                                       Should sum up to 1. 
  std::vector<double> stateTransitionProbability(const std::vector<std::string>& possible_next_state,
						 const std::string current_state,
						 const unsigned int& index);


  
 private:
  void normalizeProb(std::vector<double>& v);

  
};


/////////////////////////////////  
// Functions related to rewards
/////////////////////////////////


  // getRewardUCB
  // Upper Confidence Bound (UCB) with a time-varying beta value as described in:
  // Information-Theoretic Regret Bounds for Gaussian Process Optimization in the Bandit Setting
  // https://ieeexplore.ieee.org/document/6138914
  //
  // Inputs:  val      =  expectation
  //          var      =  variance
  //          n_cells  =  number of cells in the grid
  //          n_rounds =  number of rounds completed - Number of the times the agent
  //                      has run the search/planning problem
  //            delta  =  scale for beta - see paper for more info
  //
  // Outputs:    expected reward for that state
  
double getRewardUCB(double val, double var, double n_cells,
		    double n_rounds, double delta);



  // probf:
  // finds the probablity of a value given all the mean and sigmas found
  // so far per the Gumbel distrobution
  //
  // Inputs:   val     = value to investigate
  //           mean    = means
  //           sigma   = std deviations. 

double probf( double const &val, std::vector<double> const &mean,
	      std::vector<double> const &sigma);




  // Binary search:
  // finds the probablity value associated with the val'th percentile.
  // I.e, val = 0.25 returns the value of the 25th percentile.
  //
  // Inputs:    val     = percentile input (0,1)
  //            linespace  = discrete set of points
  //            probOfVals = associated probability of values in linespace
  //            thresh     = amount of allowable error
  //            mean       = means (see probf)
  //            sigma      = std deviations (see probf)
  //
  // Outputs:   value of the (val)th percentile

double binarySearch(double const &val, std::vector<double> const &probOfVals,
		    std::vector<double> const &linespace, double const &thresh,
		    std::vector<double> const &mean, std::vector<double> const &sigma);



  //---------------------------------
  //  calculateZStarGumbel
  //
  //     Taken from the maximum value information heuristic
  //     reward from page 108 of Victoria Preston's masters
  //     thesis,
  //     https://core.ac.uk/download/pdf/286027858.pdf
  //     and the implementation in PLUMES (mvi.py)
  //     Tried to acomplish this without linking to a lin-alg lib
  //
  //     Inputs:     M    = number of samples in the range (0,1)
  //                 vals = vector of expected values
  //                 vars = vector of variance associated with vals
  //     Outputs:    Z* distribution as a vector. 
                     
std::vector<double> calculateZStarGumbel(unsigned int const &M, std::vector<double> vals,
					 std::vector<double> vars, double robot_noise);



  // ----------------------------------------
  // getRewardMVI.
  //     Calculate the Maximum value information heuristic reward
  //     as reported in the PLUMES paper
  //     https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=8767964
  //     and
  //     https://arxiv.org/pdf/1703.01968.pdf
  //
  //     Inputs:     val   = expected value
  //                 var   = variance
  //                 zStar = Zstar distibution as calculated via
  //                         calculateZStarGumbel()
  //     Outputs:    estimated reward for that state
  //         

double getRewardMVI(double val, double var, std::vector<double> const &zStar);



#endif
