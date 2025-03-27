/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SimpleAStar.h                                   */
/*    DATE: Jan 28th, 2022                                  */
/************************************************************/

#ifndef SIMPLEASTAR_HEADER
#define SIMPLEASTAR_HEADER


#include <vector>
#include <map>
#include <set>
#include <math.h>
#include <list>
#include <iostream>
#include <algorithm>    // std::reverse
#include <limits>
#include "GraphNode.h"



class SimpleAStar
{
 public:
  SimpleAStar();
  ~SimpleAStar();

  typedef std::vector<double> vertex;
  typedef std::size_t index;
  typedef std::pair<index,index> edge;
  typedef std::vector<index> path;


  //----------------------------------
  // This is the main function that should be called
  //      Check above for the typedef ^^
  //      Inputs: V = a map of indexes to their state
  //                  Ex.  V[1] = {1,2,3} could represent
  //                  vertex 1 has a state (or position)
  //                  x = 1, y = 2, z = 3;
  //              E = a vector of edges
  //                  Ex. E[1].first = 1, and E[1].second = 2
  //                  would represent an edge exists from 
  //                  vertex 1 to 2.
  //              Obs = a set of indices that are to be
  //                  considered obstacles.
  //                  Ex. Obs.contains(3) would return
  //                  true if the third index is considered
  //                  an obstacle.
  //              idx_start = a set of acceptable
  //                  starting indices
  //              idx_final = a set of acceptable
  //                  final indices,
  //       
  //      Output: path = sequence of indices from start
  //                  to finish. 
  path  searchPath( const std::map<index, vertex>& V,
		     const std::vector<edge>& E,
		     const std::set<index>& Obs,
		     const std::set<index>& idx_start,
		     const std::set<index>& idx_final );


  

  // For faster performance, you can preload the graph
  // using preload_graph(...) and then one can repeatedly
  // use search_path_fast(...) which can be faster.
  // The assumption is that the graph - all the inputs to
  // preload_graph - do not change over time.
  // The inputs are the same as described in search_path(...)
  bool preloadGraph(const std::map<index, vertex>& V,
		    const std::vector<edge>& E,
		    const std::set<index>& idx_start,
		    const std::set<index>& idx_final,
		    bool asc_cell_id = true);
  
  path searchPathFast(const std::set<index>& Obs);
  

  double calcCost( const vertex& Vi, const vertex& q);
  double calcHeuristic( const vertex& Vi, const vertex& q);
  

 protected:


 private:

  double minHeuristicToGoal(const std::map<index, vertex>& V,
			    const int node_number,
			    const std::set<index>& idx_final);
  
  path  recoverPath(const std::list<Node>& closed_list,
		    const std::set<index>& idx_start,
		    index idx_final);
  
  path  recoverPathFast(index idx_final);

  //  Member variable to hold the info about the graph
  //  A map should be fast enough per
  //  https://www.codeproject.com/Articles/866996/Fast-Implementations-of-Maps-with-Integer-Keys-in
  std::map<index, Node> m_node_map;
  std::list<index> m_open_list_seed;

  bool m_graph_preloaded;
  

    
};
  




#endif 
