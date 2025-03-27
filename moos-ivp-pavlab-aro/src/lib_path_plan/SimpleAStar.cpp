/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleAStar.cpp                                 */
/*    DATE: Jan 28th, 2022                                  */
/************************************************************/

/* This class defines a simple AStar algorithm for general
   use.  
   
   To use, one must specify the cost and heuristic in 
   calcCost and calcHeuristic.

*/


#include "SimpleAStar.h"

//---------------------------------------------------------
// Constructor

SimpleAStar::SimpleAStar()
{
  m_graph_preloaded = false;
}


//---------------------------------------------------------
// Destructor

SimpleAStar::~SimpleAStar()
{
}



// define a cost function
double SimpleAStar::calcCost( const SimpleAStar::vertex& Vi, const SimpleAStar::vertex& q)
{
  double distance = 0;
  // Use Euclidian distance for now
  for (int j=0; j<Vi.size(); j++) {
    double ds = Vi[j] - q[j];
    distance +=  ds*ds;
  }
  return (sqrt(distance));
}

// define a heuristic function
double SimpleAStar::calcHeuristic( const SimpleAStar::vertex& Vi, const SimpleAStar::vertex& q)
{
  double factor = 1.0;
  double cost = calcCost(Vi, q);
  
  return( factor * cost);

}

// Calculate min heuristic in the case that there are multiple goals indices 
double SimpleAStar::minHeuristicToGoal(const std::map<SimpleAStar::index, SimpleAStar::vertex>& V,
				       const int node_number,
				       const std::set<SimpleAStar::index>& idx_final)
{

  // Find the estimated cost to the closest goal.
  std::set<SimpleAStar::index>::const_iterator it; 
  double h_min = std::numeric_limits<double>::infinity();  // reset the minimum cost for this list
  for (it=idx_final.begin(); it !=idx_final.end(); ++it) {
    double h = calcHeuristic(V.at(node_number), V.at(*it));
    if (h<h_min) {
      // This is the new lowest cost to get to one of the goals.
      h_min = h; 
    }
  }
  return(h_min);
}


// define a function to find a path back
//               closed_list is a list of visited nodes, including the
//               final node, recorded as idx_final.
//               idx_final is the index of the final node that was reached
//               idx_start is a list of possible start nodes. 
SimpleAStar::path SimpleAStar::recoverPath(const std::list<Node>& closed_list,
					   const std::set<SimpleAStar::index>& idx_start,
					   SimpleAStar::index idx_final)
{
  // Define the vector "path_forward" to hold the indexes from start to goal.
  // This will be reversed to describe the path from start to finish. 
   std::vector<SimpleAStar::index> path_forward;

   // Start the path_forward vector with idx_final
   path_forward.push_back(idx_final);
   
   // Loop through nodes following the parent node field to get back to the start.
   SimpleAStar::index step = idx_final;

   // keep looping until the current step is in the set of starting indices
   while ( idx_start.count(step) == 0 ) {
     // find the node in the closed list that is the parent of the node that is equal to "step"

     // loop through closed list to find the node named "step"
     std::list<Node>::const_iterator it;  
     for (it=closed_list.begin(); it !=closed_list.end(); ++it) {
       if (it->getNumber() == step) {
	 // found node at this step in the path. Get the node listed in the parent field
	 // and set it to be the new step.  Add it to the front of path_forward and continue on.
	 step = it->getParent();
	 path_forward.push_back(step);
	 break;
       }
     }  // end of the it3 loop
   } // end of while loop
   
   // Reverse the path
   std::reverse(path_forward.begin(), path_forward.end());
   
   /*
   std::cout << path_forward.size() << " nodes found between the start and the goal. "<< std::endl;
   std::cout << "They are: start config -> ";
   for (int k=0; k<=(path_forward.size()-1); k++) {
     std::cout << path_forward[k] << " -> "; 
   }
   std::cout << "goal config" << std::endl;
   */
   return path_forward;

}


//-----------------------------------------------------
// AStar search function.
// See header for inputs

SimpleAStar::path SimpleAStar::searchPath( const std::map<SimpleAStar::index, SimpleAStar::vertex>& V,
					    const std::vector<SimpleAStar::edge>& E,
					    const std::set<SimpleAStar::index>& Obs,
					    const std::set<SimpleAStar::index>& idx_start,
					    const std::set<SimpleAStar::index>& idx_final )
{

  // A* algorithm. Following the turtorial here: http://web.mit.edu/eranki/www/tutorials/search/

  // Step 0: Book keeping
  // initialize the open list to hold all the nodes on the "frontier", or the next ones
  // to check. 
  std::list<Node> open_list;

  // initialize the closed list to hold all the nodes that are on the "interior", or all
  // the nodes that have been checked already.
  std::list<Node> closed_list;

  // initialize list to hold all the nodes found near "q".  This gets erased and filled
  // up each time a new "q" is found.
  std::list<Node>  neighbors_list;  //list to hold all the nodes found near q.

  // Place all of the starting nodes on the open list.
  // if they are NOT obstacles
  // It does not matter which order
  // Set the total cost value "f" to be 0
  std::set<SimpleAStar::index>::const_iterator s_it1;
  for (s_it1=idx_start.begin(); s_it1 !=idx_start.end(); ++s_it1) {
    if (Obs.count(*s_it1) == 0) {
      Node start_node;
      start_node.setNumber(*s_it1);
      // There is no parent node here.
      start_node.setH(minHeuristicToGoal(V, start_node.getNumber(), idx_final));
      start_node.setF(start_node.getH());
      start_node.setG(0);
      open_list.push_back(start_node);
    }
  }

  Node q;  // current position node used in the algorithm
  Node working_node;  // working node used in the algorithm
  Node neighbor_node;  // working node used in the algorithm
    

  // Loop through each node on the frontier (open_list).
  while ( not open_list.empty()) {
    
    // Step 1. 
    // find the node with the smallest "f" on the open list and call it q
    std::list<Node>::iterator it;  // define the iterator for the open list
    double f_min = std::numeric_limits<double>::infinity();  // reset the minimum cost for this list

    for (it=open_list.begin(); it != open_list.end(); ++it) {

      // Compare the cost ("f") for this node to f_min
      if (it->getF() < f_min) {

	// This node has a lower cost. Set this to the new min
	f_min = it->getF();
	// set this node to "q" or the working node.  There is probably a better way to do this...
	q = *it;
	
      }
    }

    // And remove "q" from the open list
    for (it=open_list.begin(); it !=open_list.end(); ++it) {
      if (it->getNumber() == q.getNumber()) {
	it = open_list.erase(it);
	break;
      }
    }

    // Step 2
    // Find the neigbor nodes (successors) as listed in the vector of edges "E".
    neighbors_list.clear();  // Clear out the list of neighbors.
    
    // Loop through the vector E to find all the edges that start or end with q.number
    for (int k=0; k<E.size(); k++) {
      bool cond1 = (E[k].first == q.getNumber() );
      bool cond2 = (E[k].second == q.getNumber() );
      if ( cond1 or cond2) {
	// Check if this neighbor node is an obstacle
	// if so, skip it.
	if ( (Obs.count( E[k].second ) == 1) or (Obs.count( E[k].first ) == 1) ) {
	  continue;
	}
	// Check if this neighbor node is a starting node,
	// If so, then skip it, since all starting nodes
	// start on the open list.
	if (cond1 and (idx_start.count(E[k].second) >0))  {
	    continue;
	} else if (cond2 and (idx_start.count(E[k].first) >0)) {
	    continue;
	}
	
	// found a neighbor node that is not an obstacle or
	// a start node
	// Add it to the list and set the parent = q.number
	if (cond1) {
	  working_node.setNumber(E[k].second);
	} else if (cond2) {
	  working_node.setNumber(E[k].first);
	}
	working_node.setParent(q.getNumber() );
	working_node.setF(-1);
	working_node.setG(-1);
	working_node.setH(-1);
	
	neighbors_list.push_back(working_node);
      }
    }
    
    // Step 3
    // Loop through each neighbor (successor)
    for (it=neighbors_list.begin(); it !=neighbors_list.end(); ++it) {

      // Step 3.0 is already taken care of in Step 2 above
      
      // Step 3.1 First check if this heighbor is in the set of goal
      // indices.
      if (idx_final.count(it->getNumber() ) > 0) {
	// Goal reached.  End and recover path

	// find the best goal node if here are more than one
	std::list<Node>::iterator neighbor_goal_check_itr;
	double g_min_to_goal = std::numeric_limits<double>::infinity();  // min cost to goal
	std::list<Node>::iterator best_goal_node_itr;
	for ( neighbor_goal_check_itr=neighbors_list.begin();  neighbor_goal_check_itr !=neighbors_list.end(); ++neighbor_goal_check_itr) {
	  
	  if ((idx_final.count(neighbor_goal_check_itr->getNumber() ) > 0) and (Obs.count(neighbor_goal_check_itr->getNumber()) == 0 )) {
	    // Compare the cost ("f") for this node to f_min
	    double this_node_g = calcCost( V.at(neighbor_goal_check_itr->getNumber() ), V.at(q.getNumber() ));
	    if (this_node_g < g_min_to_goal) {
	      // This node has a lower cost. Set this to the new min
	      g_min_to_goal = this_node_g;
	      // set this node to "q" or the working node.  There is probably a better way to do this
	      best_goal_node_itr = neighbor_goal_check_itr;
	    }
	  }
	}
	
        // First push q on the closed set.   
	closed_list.push_back(q);
	// Also push on the goal node
	closed_list.push_back(*best_goal_node_itr);

	// recover path with the closed list, the set of starting
	// indices, and the number of the final index.
	SimpleAStar::path path = recoverPath(closed_list, idx_start, best_goal_node_itr->getNumber() );
	return path;
      }	  


      
      // Step 3.2
      // This is not the goal. Compute the cost to this node.
      neighbor_node.resetNode();
      neighbor_node.setNumber( it->getNumber() );
      neighbor_node.setParent( it->getParent() );
      // if this is in the starting zone (idx_start) then the cost
      // is zero, and skip it, since it is already on the open list
      if (idx_start.count(neighbor_node.getNumber()) > 0) {
	continue;
      } else {
	neighbor_node.setG( q.getG() + calcCost( V.at(neighbor_node.getNumber() ), V.at(q.getNumber() ) ) );
      }
      // find the min heuristic to reach any of the goal indices
      neighbor_node.setH( minHeuristicToGoal(V, neighbor_node.getNumber(), idx_final) );
      neighbor_node.setF( neighbor_node.getG() + neighbor_node.getH() );

      // Step 3.3
      // Check to see if this node is on the open list.  If it is, skip adding it if the
      // calculated "f" value is highter than what is in the open list.
      // This is like checking if any "sideways" movement on the frontier is benificial.

      // loop through the open list
      std::list<Node>::iterator it2;  // define another iterator for the open and closed list
      bool check_closed_list_for_node = true;
      for (it2=open_list.begin(); it2 !=open_list.end(); ++it2) {
       
        if (it2->getNumber() == neighbor_node.getNumber() ) {
	  // node was found
	  if(it2->getF() > neighbor_node.getF() ) {
	    // this new path is less costly than what was found previously,
	    // update it here
	    //it2 = open_list.erase(it2);
	    //open_list.push_back(neighbor_node);
	    *it2 = neighbor_node;
	  }
	  // The node in question has been found and the cost evaluated.
	  // no need to check the closed list, we can move on
	  check_closed_list_for_node = false;
	  break;
	}   
      }

      // Step 3.4
      // Check to see if this node is on the closed list.  If it is, skip adding it if the
      // calculated "f" value is higher than what is in the closed list.
      // This is like chcking if any "backtracking" into the interior is benificial.

      // Only do this if we did not find the node in the open list, which is flagged
      // using the add_node boolean. Otherwise skip it for time.  The node should
      // never be on both the open and the closed list.
      bool found_node_in_closed_list = false;
      if (check_closed_list_for_node){
	for (it2=closed_list.begin(); it2 !=closed_list.end(); ++it2) {
	  
	  if (it2->getNumber() == neighbor_node.getNumber() ) {
	    // node was found
	    if(it2->getF() > neighbor_node.getF() ) {
	      // this new path is less costly than what was found previously.
	      // remove the old record from the closed list because we will
	      // add the new path to the open list
	      it2 = closed_list.erase(it2);
	      open_list.push_back(neighbor_node);
	    }

	    // we found this node on the closed list, and updated it if needed
	    found_node_in_closed_list = true;
	    break;
	  }   
	} 
      }
      // Step 3.5
      // Add the neighbor to the open list if it is still the least costly path
      // or it hasn't been found on either list yet.
      if (check_closed_list_for_node and (not found_node_in_closed_list)) {
        open_list.push_back(neighbor_node);
      }
      
    } // end of for loop for each neighbor

    // Step 4
    // We have processed all the neighbors of this node "q".
    // Push q on the closed set
    closed_list.push_back(q);
    

  } // end of while open list is not empty loop 

  // Step 5. 
  // return failure if gotten this far
  // The open list is empty and there is nothing more to check. 
  SimpleAStar::path empty_path;
  return empty_path;
}





//---------------------------------------------------
//  recoverPathFast.
//  private function used with the preloadGraph and searchPathFast
//  functions.  Same idea, but it uses the preassembled graph

SimpleAStar::path  SimpleAStar::recoverPathFast(index idx_final)
{

  // Define the vector "path_forward" to hold the indexes from start to goal.
  // This will be reversed to describe the path from start to finish. 
  std::vector<SimpleAStar::index> path_forward;

  // Start the path_forward vector with idx_final
  path_forward.push_back(idx_final);
   
  // Loop through nodes following the parent node field to get back to the start.
  SimpleAStar::index step = idx_final;
  
  // keep looping until the current step is in the set of starting indices
  while ( not m_node_map.at(step).isStart() ) {
    // find the node that is the parent node of this node  set it to be the new step.
    //Add it to the front of path_forward and continue on.
    if (m_node_map.find(step) == m_node_map.end()){
      std::vector<SimpleAStar::index> empty_path_forward;
      return empty_path_forward;
    }
    step = m_node_map.at(step).getParent();
    path_forward.push_back(step);
  } // end of while loop
  
  // Reverse the path
  std::reverse(path_forward.begin(), path_forward.end());
  
  /*
  std::cout << path_forward.size() << " nodes found between the start and the goal. "<< std::endl;
  std::cout << "They are: start config -> ";
  for (int k=0; k<=(path_forward.size()-1); k++) {
    std::cout << path_forward[k] << " -> "; 
  }
  std::cout << "goal config" << std::endl;
  */
  
  return path_forward;
  
}


//-----------------------------------------------------
// preload AStar search graph function.
// See header for inputs

bool SimpleAStar::preloadGraph(const std::map<SimpleAStar::index, SimpleAStar::vertex>& V,
			       const std::vector<SimpleAStar::edge>& E,
			       const std::set<SimpleAStar::index>& idx_start,
			       const std::set<SimpleAStar::index>& idx_final,
			       bool asc_cell_id)
{

  // Bookkeeping:
  // Clear the map and open list seed to prevent problems when running the
  // preloadGraph procedure repeatedly
  m_node_map.clear();
  m_open_list_seed.clear();
  
  
  // Create a node for each vertex in the graph
  std::map<SimpleAStar::index, SimpleAStar::vertex >::const_iterator it;

  for (it = V.begin(); it != V.end(); it++){
    Node new_node;
    new_node.setNumber(it->first);
    // parent is unknown for now
    new_node.setG(0.0);
    // total estimated cost f is not known at this time
    new_node.setH( minHeuristicToGoal(V, new_node.getNumber(), idx_final) );
    
    // find all neighbors in the list of edges
    std::vector<SimpleAStar::index> possible_neighbors;
    
    // Loop through the vector E to find all the edges that start or end with q.number
    for (int k=0; k<E.size(); k++) {
      if (E[k].first == new_node.getNumber() ) {
	possible_neighbors.push_back(E[k].second);
	
      } else if (E[k].second == new_node.getNumber() ) {	
	  possible_neighbors.push_back(E[k].first);  
      }
    }
    
    // And add all the neighbors we have found to the node
    new_node.setNeighbors(possible_neighbors);
    
    new_node.setPos(it->second);
    
    if (idx_final.count(new_node.getNumber() ) >  0) {
      new_node.setIsGoal(true);
    }
    if (idx_start.count(new_node.getNumber() ) > 0) {
      new_node.setIsStart(true);
      // total estimated cost g is exactly zero
      // so f = h for the starting nodes
      new_node.setF(new_node.getH() );
    }

    // Add this node to the map
    m_node_map[it->first] = new_node;
  }
  
  // Also save the start indices to a list to seed the
  // open list
  if (asc_cell_id) {
    std::set<SimpleAStar::index>::const_iterator it2;
    for (it2 = idx_start.begin(); it2 != idx_start.end(); it2++){
      m_open_list_seed.push_back(*it2);
    }
    
  } else {
    // reverse order
    std::set<SimpleAStar::index>::const_reverse_iterator rit2;
    for (rit2 = idx_start.rbegin(); rit2 != idx_start.rend(); rit2++){
      m_open_list_seed.push_back(*rit2);
    }
  }

  // mark as preloaded graph
  m_graph_preloaded = true;
  
  return(true);
}


//----------------------------------------------------
// searchPathFast uses the preloaded graph map
// to speed up searching.

SimpleAStar::path SimpleAStar::searchPathFast(const std::set<index>& Obs)
{

   // Faster A* algorithm. Based on the structure of the "slower" A* above.
  if (not m_graph_preloaded) {
    SimpleAStar::path no_path;
    return(no_path);
  }
  
  // Step 0: Book keeping
  // initialize the open list to hold all the nodes on the "frontier", or the next ones
  // to check. Initialize it to be the starting values that are not obstables.
  std::list<SimpleAStar::index> open_list = m_open_list_seed;
  std::list<SimpleAStar::index>::iterator itr_start_obs_check;  //
  for (itr_start_obs_check=open_list.begin(); itr_start_obs_check != open_list.end();) {
    if (Obs.count(*itr_start_obs_check) > 0 )
      itr_start_obs_check = m_open_list_seed.erase(itr_start_obs_check);
    else
      ++itr_start_obs_check;
  }

  // initialize the closed list to hold all the nodes that are on the "interior", or all
  // the nodes that have been checked already.
  std::list<SimpleAStar::index> closed_list;

  // initialize list to hold all the nodes found near "q".  This gets erased and filled
  // up each time a new "q" is found.
  std::vector<SimpleAStar::index>  neighbors_vec;  //vec to hold all the nodes found near q.

  // index variable to hold the current node we are exploring.
  SimpleAStar::index q;

  // Loop through each node on the frontier (open_list).
  while ( not open_list.empty()) {
    
    // Step 1. 
    // find the node with the smallest "f" on the open list and call it q
    std::list<SimpleAStar::index>::iterator it;  // define the iterator for the open list
    double f_min = std::numeric_limits<double>::infinity();  // reset the minimum cost for this list

    for (it=open_list.begin(); it != open_list.end(); ++it) {
      // Compare the cost ("f") for this node to f_min
      double this_node_f = m_node_map.at(*it).getF();
      if (this_node_f < f_min) {
	// This node has a lower cost. Set this to the new min
	f_min = this_node_f;
	// set this node to "q" or the working node.  There is probably a better way to do this
	q = *it;
      }
    }
    
    // And remove "q" from the open list since we will explore it.
    open_list.remove(q);

    
    // Step 2
    // Find the neigbor nodes (successors) as listed in the vector of edges "E".
    neighbors_vec = m_node_map.at(q).getNeighbors(); 

    // Step 3
    // Loop through each neighbor (successor)
    std::vector<SimpleAStar::index>::iterator neighbor_itr;
    for (neighbor_itr=neighbors_vec.begin(); neighbor_itr !=neighbors_vec.end(); ++neighbor_itr) {

      // Step 3.0 First check if this neighbor node is an obstacle
      // Check if this neighbor node is an obstacle
      // if so, skip it.
      if ( Obs.count( *neighbor_itr ) == 1 ) {
	continue;
      }

      // Step 3.1 Then check if this heighbor is a goal index
      if (m_node_map.at(*neighbor_itr).isGoal() ) {
	// Goal reached.  End and recover path

	// find the best goal node if here are more than one
	std::vector<SimpleAStar::index>::iterator neighbor_goal_check_itr;
	double g_min_to_goal = std::numeric_limits<double>::infinity();  // min cost to goal
	index best_goal_node = *neighbor_itr;
	for ( neighbor_goal_check_itr=neighbors_vec.begin();  neighbor_goal_check_itr !=neighbors_vec.end(); ++neighbor_goal_check_itr) {
	  if ((m_node_map.at(*neighbor_goal_check_itr).isGoal() ) and (Obs.count(*neighbor_goal_check_itr) == 0 )) {
	    // Compare the cost ("f") for this node to f_min
	    double this_node_g = calcCost(m_node_map.at(*neighbor_goal_check_itr).getPos(), m_node_map.at(q).getPos() );
	    if (this_node_g < g_min_to_goal) {
	      // This node has a lower cost. Set this to the new min
	      g_min_to_goal = this_node_g;
	      // record this as the best node
	      best_goal_node = *neighbor_goal_check_itr;
	    }
	  }
	}

	// for bookkeeping
        // First push q on the closed set.   
	closed_list.push_back(q);
	// Also push on the goal node
	closed_list.push_back(best_goal_node);

	// record that this neighbor's parent is q
	m_node_map.at(best_goal_node).setParent(q);
	// recover path with the closed list, the set of starting
	// indices, and the number of the final index.
	SimpleAStar::path path = recoverPathFast(best_goal_node);
	return path;
      }	  

      // Step 3.2
      // This is not the goal. Compute a proposed cost to this node
      // if we take this path.
      double proposed_cost_to_this_node = 0.0;
      // if this is in the starting zone (idx_start) then the g cost
      // is always zero and no need to update because it always
      // starts on the open list. 
      double g_along_this_path = 0.0;
      if (m_node_map.at(*neighbor_itr).isStart() ) {
	continue;
      } else {
	g_along_this_path = m_node_map.at(q).getG() + calcCost(m_node_map.at(*neighbor_itr).getPos(), m_node_map.at(q).getPos() );
	proposed_cost_to_this_node = g_along_this_path + m_node_map.at(*neighbor_itr).getH();
      }

 
      // Step 3.3
      // Check to see if this neighboring node is on the open list.  If it is,
      // only update with f value (and parent node) if the proposed cost is less than
      // what was estimated prevously.
      // This is like checking if any "sideways" movement on the frontier is benificial.
      bool check_closed_list_for_node = true;
      
      // first loop through the open list
      std::list<SimpleAStar::index>::iterator open_itr;  
      for (open_itr=open_list.begin(); open_itr !=open_list.end(); ++open_itr) {

	if ( *neighbor_itr == *open_itr) {
	  //neighbor is on the open list
	  
	  if (proposed_cost_to_this_node < m_node_map.at(*open_itr).getF() ) {
	    // the new path is better.
	    m_node_map.at(*open_itr).setF(proposed_cost_to_this_node);
	    m_node_map.at(*open_itr).setG(g_along_this_path);
	    m_node_map.at(*open_itr).setParent(q);
	  }
	  // we found this node on the open list, and updated it if needed.
	  check_closed_list_for_node = false;
	  break;
	}
      }

      // Step 3.4
      // Check to see if this neighboring node is on the closed list.
      // If it is, only update with with new f value (and parent node)
      // if the proposed cost is less than what was estimated previously. 
      // This is like chcking if any "backtracking" into the interior is benificial.

      // Only do this if we did not find the node in the open list, which is flagged
      // using the add_node boolean. Otherwise skip it for time.  The node should
      // never be on both the open and the closed list.
      bool found_node_in_closed_list = false;
      if (check_closed_list_for_node) {
	std::list<SimpleAStar::index>::iterator closed_itr;
	for (closed_itr=closed_list.begin(); closed_itr !=closed_list.end(); ++closed_itr) {
	  
	  if ( *neighbor_itr == *closed_itr) {
	    //neighbor is on the closed list
	    if (proposed_cost_to_this_node < m_node_map.at(*closed_itr).getF() ) {
	      // the new path is better.
	      m_node_map.at(*closed_itr).setF(proposed_cost_to_this_node);
	      m_node_map.at(*closed_itr).setG(g_along_this_path);
	      m_node_map.at(*closed_itr).setParent(q);
	      
	      // now move this node index back onto the open list
	      // here we will erase it from the closed list,
	      closed_itr = closed_list.erase(closed_itr);
	      open_list.push_back(*neighbor_itr);

	    }
	    // we found this node on the closed list, and updated it if needed
	    found_node_in_closed_list = true;
	    break;
	  }
	} // end of for loop for closed list
      }

      // Step 3.5
      // Add the node to the open list if we didn't find it already in
      // the open list (marked by the flag check_closed_list_for_node)
      // AND did not find it in the closed list (marked by the
      // flag found_node_in_closed_list).
      if (check_closed_list_for_node and (not found_node_in_closed_list)) {
	m_node_map.at(*neighbor_itr).setParent(q);
	m_node_map.at(*neighbor_itr).setG(g_along_this_path);
	m_node_map.at(*neighbor_itr).setF(proposed_cost_to_this_node);
	open_list.push_back(*neighbor_itr);

      }
      
    } // end of for loop for each neighbor node.

    // Step 4
    // We have processed all the neighbors of this node "q".
    // Push q on the closed set
    closed_list.push_back(q);


  } // end of while open list is not empty


  // Step 5. 
  // return failure if gotten this far
  // The open list is empty and there is nothing more to check. 
  std::cout << "FAILURE. Could not find a path" << std::endl;
  SimpleAStar::path empty_path;
  return empty_path;
}





