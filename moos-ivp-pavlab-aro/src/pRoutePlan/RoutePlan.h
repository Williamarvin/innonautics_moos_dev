/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RoutePlan.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef RoutePlan_HEADER
#define RoutePlan_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MBUtils.h"
#include "EsriBathyGrid.h"
#include "SimpleAStar.h"
#include "XYCircle.h"
#include "NodeMessage.h"
#include <limits>  // for costCalc
#include "PathUtils.h"  // for greedy schedule
#include "GeomUtils.h"  // for distPointToSeg
#include <algorithm> // for reverse
class RoutePlan : public AppCastingMOOSApp
{
 public:
   RoutePlan();
   ~RoutePlan();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
  //std::string getGridLabel(std::string grid_spec);


 private:
   void plotPath(std::vector<size_t> path);
   void plotStartIds();
   void plotEndIds();
 
   void updateWaypointPath(std::vector<size_t> path);
   std::vector<size_t> findInitialExplorePath();
   void updateStartOrEndPaths();

   // Proposal proceedures
   bool handlePathProposal(std::string msg, double msg_time);
   std::string getPathProposalSpec(std::vector<size_t> path, double cost);
   bool checkProposalsForWin();
   std::set<std::size_t> getVerticesFromValidPaths() const;
   double calcCost(const std::vector<double>& v1,
		   const std::vector<std::vector<double> >& V2,
		   std::size_t& index_of_closest_vertex);
   void sendOutOwnProposal(std::string spec);
   std::vector<std::size_t> findBestPathDirection();

   // Path proceedures
   double calcDynamicThreshold() const;

   bool isViablePath(std::vector<std::size_t> path);
   bool isPathShorter(std::vector<size_t> const &path1, std::vector<size_t> const &path2);

   
 private: // Configuration variables

   // Proposal
   std::string m_proposal_var_name = "PROPOSED_PATH";

   // consensus time params
   double m_consensus_wait_time = 5.0;

   // proposal params
   double m_proposal_wait_time = 2.0;
   unsigned int m_max_proposal_iterations = 10;

   
   std::map<std::string, std::string> m_path_color_map;
   double m_no_data_value = 0;
   double m_lat_origin = 0;
   double m_lon_origin = 0;

   // Path obstacle thresholds. 
   double m_depth_threshold = 15;
   double m_var_threshold = 0.005;
   double m_var_pct_thresh = 0.3;

   // Path config variable 
   bool m_use_all_top_cells_as_start = false;
   bool m_use_all_bottom_cells_as_goal = false;
   int  m_number_of_vehicles = 1;
   int  m_vehicle_n = 0;
   std::string m_vname;

   // Timeout variables
   bool m_timeout = false;
   int m_timeout_time = 8000;


 private: // State variables
   
   EsriBathyGrid m_grid;

   // timers
   double m_time_last_grid_was_received = 0.0;
   double m_start_time = -1;
   double m_time_to_first_path = -1;

   
   // counters 
   int m_full_grid_rcvd = 0;
   int m_full_grid_deltas_rcvd = 0;
   unsigned int m_no_path_counter = 0;
   unsigned int m_proposal_timeout_cnt = 0;
   unsigned int m_didnt_win_counter = 0;
   
   // flags
   bool m_final_path_found = false;
   bool m_surveying = false;
   bool m_first_grid_received = false;
   bool m_new_grid_received = false;
   bool m_explore_possible_paths = false;
   bool m_sent_final_path_found = false;

   // A* variables
   SimpleAStar m_astar = SimpleAStar();
   std::map<size_t, std::vector<double> > m_vertices;
   std::vector<std::pair<size_t,size_t> > m_edges;
   std::set<size_t> m_obstacles;
   std::set<size_t> m_final_obstacles;
   
   std::set<size_t> m_start_ids;
   std::set<size_t> m_end_ids;

   std::vector<size_t> m_p;
   std::vector<size_t> m_rev_path;
   std::vector<size_t> m_fwd_path; 

   std::vector<XYCircle> m_active_circles;
   std::vector<XYCircle> m_active_start;
   std::vector<XYCircle> m_active_end;


   // Proposal variables
   unsigned int m_proposal_iterations = 0;
   double m_last_proposal_time = 0.0;
   bool m_won_a_proposal = false;
   bool m_found_path_that_may_be_viable = false;
   bool m_first_proposal_completed = false;
   bool m_first_proposal_won = false;
   unsigned int m_path_maintained_cnt = 0;
   unsigned int m_path_maintained_max = 5; 

   double m_nav_x = 0.0;
   double m_nav_y = 0.0;
   std::map<std::string, std::vector<std::size_t> > m_proposed_paths;
   std::map<std::string, double> m_proposed_costs;
   std::map<std::string, double> m_proposal_arrival_time;

   std::vector<std::size_t> m_final_path;
   
   

  
};

#endif 
