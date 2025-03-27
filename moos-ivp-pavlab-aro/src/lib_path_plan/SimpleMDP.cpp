/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleMDP.cpp                                   */
/*    DATE: March 7th, 2022                                 */
/************************************************************/

/* This class defines several algorithms related to 
    Markov Decision Processes, and reward functions. 
   
   To use, one must specify the probability of state transitions
   which are unique to the specific problem format. 
   In this case change the implementation of 
   stateTransistionProbability(...) procedure.

   Reference for implementations are "AI, A Modern Approach
   by Stuart Russell and Peter Norvig"  Chapter 17.

*/


#include "SimpleMDP.h"

//---------------------------------------------------------
// Constructor

SimpleMDP::SimpleMDP()
{
}


//---------------------------------------------------------
// Destructor
SimpleMDP::~SimpleMDP()
{
}


//-------------------------------------------------------
//  Value Iteration (Figure 17.4) in AI: A Modern Approach 3rd Ed
//                         see header file for inputs and outputs. 
std::map<std::string, double> SimpleMDP::valueIteration(const std::map<std::string,double>& rewards,
							const std::vector<std::pair<std::string,std::string> >& edges,
							const double& discount,
							const double& epsilon,
							const int max_iter=50)
{

  // Step 0: Initialize
  // and preload map of possible next states.
  
  // Map of possible next states
  std::map<std::string,std::vector<std::string> > next_states_map;
  // ex next_states_map["node1"] = {"node2", "node3", ... }
 
  // Utilities
  std::map<std::string,double> U;
  std::map<std::string,double> U_prime;
  // initialize delta to be slightly greater than the exit criteria
  double delta =   epsilon * (1.0 - discount) / discount + .1;

  // initialize utilies and build map
  std::map<std::string,double>::const_iterator it;
  std::vector<std::pair<std::string,std::string> >::const_iterator it2;
  for (it = rewards.begin(); it !=rewards.end(); it++) {
    U[it->first] = 0.0;
    U_prime[it->first] = 0.0;

    // find all the possible next states and load into map
    // only load unique next states.
    std::vector<std::string> actions;
    for (it2 = edges.begin(); it2 != edges.end(); it2++){
      if (it2->first == it->first) {
	if (std::find(actions.begin(), actions.end(), it2->second) == actions.end()){
	  actions.push_back(it2->second);
	}
      }
    }
    next_states_map[it->first] = actions;
  }


  // Step 1.  Main Value Iteration.  (Contraction based proof of convergence.) 
  int count = 0;
  while ((delta >= epsilon * (1.0 - discount) / discount) and (count <= max_iter)) {
    
    U = U_prime;
    delta = 0.0;

    double utility;
    for (it = U.begin(); it != U.end(); it++) {

      // loop through all the possible actions and calculate the best utility possible
      double max_next_action_utility = 0.0;
      double this_next_action_utility;

      for (unsigned int i = 0;  i<next_states_map[it->first].size(); i++) {
	std::vector<double> trans_prob = stateTransitionProbability(next_states_map[it->first],
								     it->first,i);

	// compute the utility of this next proposed action:
	this_next_action_utility = 0.0;
	for (unsigned int j = 0; j<trans_prob.size(); j++) {
	  this_next_action_utility += trans_prob[j] * U[next_states_map[it->first][j]];
	}
        
	// is this best action we've found that maximizes utility?
	if (this_next_action_utility > max_next_action_utility) {
	  max_next_action_utility = this_next_action_utility;
	}
							
      }

      // Update U_prime
      U_prime[it->first] = rewards.at(it->first) + discount * max_next_action_utility;
      
      // Update delta if needed
      double diff = std::abs(U_prime[it->first] - U[it->first]);

      if (diff > delta) {
	delta = diff;
      }
 
    } // end of loop for each state

    count++;
  }

  return(U);
}



//--------------------------------------------------
// State transition probability
//   This function will likely need to be changed via
//   the polymorphic properties to whatever model is being used.
//   See header file for more details. 

std::vector<double> SimpleMDP::stateTransitionProbability(const std::vector<std::string>& possible_next_state,
							   const std::string current_state,
							   const unsigned int& index)
{
  std::vector<double> transition_prob;
  // CUSTOM MODEL CODE GOES HERE: 
  // For the bathymetric surveying problem the state is something like
  // current_state = "1,3"  where 1 is the unique grid cell number, and
  // 3 is the heading, which is defined in EsriBathyGrid.cpp 
  // In the bathymetric surveying problem, each state only
  // can only have three possible next states. And the previous code enforced
  // uniqueness.

  bool return_default_value = true;  // Backup to be sure we return something

  int number_of_next_states = possible_next_state.size();

  // Check if one of the states has the same heading as the current state
  // first find current heading from the state description
  bool ok_state = true;
  int current_hdg = 0;
  if (current_state.length() > 0) {
    char last = current_state[current_state.length()-1];
    current_hdg = atoi(&last);
  } else {
    ok_state = false;
  }
    
  // Build a vector of all the other possible states.
  bool ok_next_states = true;
  std::vector<int> next_states_hdg;
  for (unsigned int i = 0; i<number_of_next_states; i++){
    std::string next_state = possible_next_state[i];
    if (next_state.length() > 0) {
      char last = next_state[next_state.length()-1];
      next_states_hdg.push_back( atoi(&last) );
    } else {
      ok_next_states = false;
      break;
    }
  }
  
  // Define probabilities
  if (ok_state and ok_next_states) {

    // Check if this action would result in the same heading
    // i.e. "just moving forward at this heading".  This
    // action is highly probable.
    if (current_hdg == next_states_hdg[index]) {
      
      for (unsigned int i = 0; i<possible_next_state.size(); i++) {
	if (i == index)
	  transition_prob.push_back(0.9);
	else
	  transition_prob.push_back(0.1);
      }

      normalizeProb(transition_prob);
      return_default_value = false;
    } else {

      // This action would result in a change of heading.
      // set the highest probabilty to this next state,
      // with the next highest probability to maintaining
      // the current heading (if a possible action)
      for (unsigned int i = 0; i<possible_next_state.size(); i++) {
	if (i == index)
	  transition_prob.push_back(0.7);
	else if (current_hdg == next_states_hdg[i])
	  transition_prob.push_back(0.2);
	else
	  transition_prob.push_back(0.1);
      }
      
      normalizeProb(transition_prob);
      return_default_value = false;
    }

  } // end if ok_state and ok_next_states


  // Catch all for any errors;
  if (return_default_value) {
    // default if no predetermined model exists
    for (unsigned int i = 0; i<possible_next_state.size(); i++) {
      if (i == index)
	transition_prob.push_back(1.0);
      else
	transition_prob.push_back(0.0);
    }
    
  }
  
  return(transition_prob);
}


//---------------------
// Procedure: normalizeProb

void SimpleMDP::normalizeProb(std::vector<double>& v)

{
  double sum_of_elems = std::accumulate(v.begin(), v.end(), 0.0);
  if (sum_of_elems == 0.0)
    return;
  std::vector<double>::iterator it;
  for (it = v.begin(); it != v.end(); it++) {
    *it = *it/sum_of_elems;
  }
  return;
}


// ----------------------
// Procedure:  getRewardUCB
// See header for details


double getRewardUCB(double val, double var, double n_cells,
		    double n_round, double delta)
{
 
  double beta = 2.0 * log(n_cells * pow( n_round,2) * M_PI*M_PI / ( 6.0 * delta));
  double reward = val + beta*var;  
  return (reward);
  
}


// ------------------------------------
// Procedure: probf
// See header for details

double probf( double const &val, std::vector<double> const &mean, std::vector<double> const &sigma)
{
  double gamma;
  double cdfgamma;
  double ret_val = 1.0;
  for (unsigned int i = 0; i<mean.size(); i++) {
    gamma =  (val - mean[i]) / sigma[i];
    cdfgamma = ( 0.5 * (1.0 + erf(gamma/ sqrt(2.0)) ) );
    ret_val = ret_val * cdfgamma;
  }
  return(ret_val);
}


//-------------------------------------------
// Procedure: binary search:
// See header for details

double binarySearch(double const &val, std::vector<double> const &probOfVals,
		    std::vector<double> const &linespace, double const &thresh,
		    std::vector<double> const &mean, std::vector<double> const &sigma)
{

  // Step 1.  Find the index with the closest probablity to val
  double min_diff = std::numeric_limits<double>::max();
  unsigned int idx;
  unsigned int idx_best;
  for (unsigned int idx = 0; idx<probOfVals.size(); idx++){
    if ( abs(probOfVals[idx]-val) < min_diff) {
      min_diff =  abs(probOfVals[idx]-val);
      idx_best = idx;
    }
  }
  // Step 2.  If this is good enough, just return it
  if ( min_diff < thresh)  {
    return( linespace[idx_best] );
  }

  // Step 3.  Find bounds to start the binary search.
  double left, right;
  if (( idx_best == 0) or (idx_best == ( probOfVals.size() )) ) {
    // we are at one end or the other, set accordingly to avoid SEGFAULT
    left  = linespace[idx_best];
    right = linespace[idx_best];
    
  } else if (probOfVals[idx_best] > val) {
    left  = linespace[idx_best-1];
    right = linespace[idx_best];
    
  } else  {
    left  = linespace[idx_best];
    right = linespace[idx_best+1];
  }

  // Step 4.  Iterate via binay search until converged.
  
  double mid = (left + right) / 2.0;
  double prob_of_mid = probf(mid, mean, sigma);
  int count = 0;
  int max_count = 100;
  while( (abs(prob_of_mid - val) > thresh) and (count < max_count) ) {
    if (prob_of_mid > val) 
      right = mid;
    else
      left = mid;

    mid = (left + right) /2.0;
    prob_of_mid = probf(mid, mean, sigma);
    count ++;
  }

  return(mid);
}



// ---------------------------------
//   Procedure: calculateZStarGumbel
//   See header file for definition

std::vector<double> calculateZStarGumbel(unsigned int const &M, std::vector<double> vals,
					 std::vector<double> vars, double robot_noise ) {

  // The vector to return
  std::vector<double> zStar;
  
  // Step 1. get a distibution
  // calculate sigma vector
  std::vector<double> sigma;
  for (unsigned int j=0; j<vars.size(); j++) {
    sigma.push_back( sqrt(abs(vars[j])) );
  }

  // get max val
  double max_val = *std::max_element(vals.begin(), vals.end());

  // Following from Victoria's code
  double left = max_val - 5.0 * robot_noise;
  double right;
  
  if (probf(left, vals, sigma) < 0.25){
    
    std::vector<double> mean_plus_sigma_vec;
    for (unsigned int i=0; i<vals.size(); i++) {
      mean_plus_sigma_vec.push_back(vals[i] + 5.0 * sigma[i]);
    }
    right = *std::max_element(mean_plus_sigma_vec.begin(), mean_plus_sigma_vec.end());

    int count = 0;
    while ((probf(right, vals, sigma) < 0.75) and (count < 20)) {
      right = right + right - left;
      count++;
    }

    // Create a linespace from left to right
    std::vector<double> linespace;
    int intervals = 50;
    linespace.push_back(left);
    
    for (int k=1; k<=intervals; k++){
      double k_dbl = static_cast<double>(k);
      double spaces = static_cast<double>(intervals);
      linespace.push_back(left + (right - left) * k_dbl / spaces);
    }

    // compute the probability of the elements of linespace
    std::vector<double> prob;
    for (int p=0; p<linespace.size(); p++) {
      prob.push_back( probf(linespace[p], vals, sigma) );
    }

    // Step 2. calculate a and b based on this distibution
    // run binary search for med, q1 and q3. 
    double med = binarySearch(0.50, prob, linespace, 0.01, vals, sigma);
    double q1  = binarySearch(0.25, prob, linespace, 0.01, vals, sigma);
    double q2  = binarySearch(0.75, prob, linespace, 0.01, vals, sigma);

    double b = (q1-q2) / (log(log(4.0/3.0)) - log(log(4.0)));
    double a = med + b * log(log(2.0));

    // Step 3. Calculate Z* for a uniformly distributed vector
    //         we use a deterministic method for speed.
    for (int j = 0; j<M; j++) {
      double r = static_cast<double>(j)/static_cast<double>(M) + 0.5 / static_cast<double>(M);
      zStar.push_back(a - b * log( -1.0 * log(r)) - 2.0 * robot_noise);
    } 

  } else {
    // just return a vector where every value is left
      for (unsigned int i=0; i<M; i++) {
	zStar.push_back(left);
      }
  }

  return(zStar);
    
}


// ----------------------------------------
//  Procedure: getRewardMVI.
//  See header for definition

double getRewardMVI(double depth, double var, std::vector<double> const &zStar)
{
  // and calculate the approximate reward of a query based on
  // samples of z_star
  double val = 0.0;
  for (int i = 0; i<zStar.size(); i++) {
    double gamma_z_star_i = (zStar[i] - depth) / (var);
    double phi_gamma_z_star_i = 1.0 / sqrt(2 * M_PI) * exp(-0.5 * pow(gamma_z_star_i,2));
    double PHI_gamma_z_star_i = 0.5 * (erf( gamma_z_star_i / sqrt(2.0)) + 1.0);
    
    val += gamma_z_star_i * phi_gamma_z_star_i / (2.0 * PHI_gamma_z_star_i) - log(PHI_gamma_z_star_i);
  }

  double ret_val = val/static_cast<double>(zStar.size());

  // some sanity checks:
  if ((ret_val < 0) or (std::isnan(ret_val))) {
    return(0.0);
  } else {
    return( ret_val );
  }
}
