/************************************************************/
/*    NAME: Filip Stromstad                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DemusterAssign.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef DemusterAssign_HEADER
#define DemusterAssign_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "demuster_geometry.h"
#include <map>


class DemusterAssign : public AppCastingMOOSApp
{
 public:
   DemusterAssign();
   ~DemusterAssign();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables
   std::string m_type;
   Point m_CG;
   Point m_anchor;
   double m_distance;
   double m_heading;
   std::string m_heading_type;
   double m_margin;
   double m_margin_muster;
   double m_dubin_turn_radius;

   std::string m_assignment_algorithm;
   std::string m_assignment_metric;
   double m_heading_distance_weight;
   bool m_project_first_point_by_capture_radius;
   double m_capture_radius;

   bool m_turn_in_place;

   //Type specific variables
   double m_circle_radius;
   double m_arrow_angle;
  
 private: // State variables
   bool m_calculate_formation;
   double m_formations_calculated;
   std::vector<Pose> m_formation;
   std::map<std::string, Pose> m_node_poses;
   std::map<std::string, Pose> m_node_destinations;

   std::vector<std::vector<double> > m_cost_matrix;
   double m_total_cost;

   std::vector<std::string> m_arrived_nodes; //for temp convoy solution
   std::vector<std::string> m_complete_nodes;
   double m_complete_timer;

   std::map<std::string, bool> m_node_deadlocks;
   std::vector<std::string> m_node_permablocks;

   std::map<std::string, bool> m_who_wants_control;
   int m_number_of_controlers;
   std::string m_mode;

};

#endif 
