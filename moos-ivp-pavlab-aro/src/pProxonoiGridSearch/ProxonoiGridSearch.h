/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ProxonoiGridSearch.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ProxonoiGridSearch_HEADER
#define ProxonoiGridSearch_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "NodeRecord.h"        // for node record
#include "NodeRecordUtils.h"   // for processing incoming node reports

#include "ProxGrid.h"          // for ProxGrid
#include "PathTree.h"
#include "SimpleMDP.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsConvexGrid.h"
#include "XYSegList.h"
#include "XYPolygon.h"
#include "VoronoiUtils.h"       // for vsplit, ploy
#include "GeomUtils.h"
#include "AngleUtils.h"
#include "XYPoint.h"            // for bloom record
#include "XYFormatUtilsPoint.h" // for string2Point



class ProxonoiGridSearch : public AppCastingMOOSApp
{
 public:
   ProxonoiGridSearch();
   ~ProxonoiGridSearch();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

   bool handleNodeReport(std::string msg);
   bool handleContactsList(std::string msg);
   bool handlePopStateList(std::string key, std::string msg);
   bool handleNeighborProxPoly(std::string msg);
   bool handleOwnProxPoly(std::string msg);
   bool handleSampleNotification(std::string msg);
   bool handleSampleFinished(std::string msg); 


   //  Need this to update our own path in the voronoi cell
   bool getNextPathLookahead(double curr_x, double curr_y, double curr_heading,
			     XYPolygon &region1, XYPolygon &region2, bool region_combo,
			     std::string vname, XYSegList &new_path, double &value);
  
   bool buildPathTree(int iterations, double curr_x, double curr_y, double curr_heading,
		      XYPolygon &region1, XYPolygon &region2, bool region_combo, PathTree &path);

   XYSegList getBestPath(PathTree pathtree, std::string vname, double &value);


   void updateIgnoreSet();
   void sliceOwnPolygon();
   void determineRegionConfig(XYPolygon &region2, bool &region_combo,
			      bool assume_participation, std::string contact_name);
   double calcCollectiveValue(bool assume_participation);

   bool didAgentJustNotifyToSample(std::string vname, double stale_thresh);
   
   
 protected:
   void registerVariables();
   
 private: // Configuration variables
   std::set<std::string> m_valid_option_set;
   bool m_post_grid;
   int  m_grid_posts_skipped;
   int  m_grid_post_skip_interval;

   double m_cool_grid_value;
   double m_cool_grid_variance;
   double m_cool_grid_interval;
   double m_last_time_grid_was_cooled;

   // Custom things
   int m_tree_depth;
   int m_max_mdp_iter;
   double m_discount_factor;
   double m_cleared_grid_reward;
   double m_cleared_grid_variance;
   double m_sensor_meas_variance;
   double m_no_detection_in_cell_reward;
   double m_no_detection_in_cell_variance;
   double m_timeout_after_sampling;
   double m_time_last_sample_finished; 
   
   
   
 private: // State variables
   double m_nav_x;
   double m_nav_y;
   double m_nav_hdg;
   double m_nav_spd;
   double m_sensor_meas;
   std::string m_vname;
   std::string m_curr_option;

   // Multi-agent book keeping
   std::map<std::string, NodeRecord> m_node_rec_map;
   std::set<std::string> m_contacts;


   std::map<std::string, std::map<std::string, double> > m_utilities;

   double m_own_max_value;
   double m_last_value;

   // The grid of where vehicles have been recently. 
   ProxGrid m_prox_grid;

   XYPolygon m_own_prox_poly;
   std::map<std::string, XYPolygon> m_neighbors_prox_poly;
   std::map<std::string,XYPolygon> m_own_poly_sliced_pieces;

  // set of contacts that are so far away that we can ignore
  // them in the marginal set calc.
  std::set<std::string> m_ignore_contact_set;

  // map of seglists posted for other contacts.
  // these will be cleared if these contacts ever get on the
  // ignore list
  std::map<std::string,XYSegList> m_alternate_best_paths_for_contacts; 

   // Map to hold the population state:
   // Key = option name
   // Val = set of names of agents for that option.
   //       some sets could be empty!
   std::map<std::string, std::set<std::string>> m_pop_state_map;

   // Map to hold the last time a sample was detected
   // by ownship or neighbors
   // Key = name
   // Val = Time last notification to sample was recieved
   std::map<std::string, double> m_last_sample_notification_time_map;

  
};

#endif 
