/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: AqctNodeCtrl.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef AqctNodeCtrl_HEADER
#define AqctNodeCtrl_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "XYFormatUtilsPoly.h" // for string2poly
#include "XYPolygon.h"         // for region

#include "XYFormatUtilsPoint.h" // for string2point
#include "XYPoint.h"            // for flag point

#include "NodeRecord.h"        // for node record
#include "NodeRecordUtils.h"   // for processing incoming node reports

#include "NodeMessage.h"       // for node messages

#include <limits>            // for std::numeric_limits<double>::max();
#include "MBUtils.h"
#include "GeomUtils.h"         // for distPointToPoint
#include "AngleUtils.h"        // for relAng


class AqctNodeCtrl : public AppCastingMOOSApp
{
 public:
   AqctNodeCtrl();
   ~AqctNodeCtrl();

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
   bool handleFlagSummary(std::string msg);
   bool handleTaggedList(std::string msg);
   bool handleIntruderTasked(std::string msg);
   void calculateInputs();
   void publishInputs();
   void updateBlockingBehavior();
   void updateDefense();

   bool calcIfClosestTo(std::string contact);
   double getCostToIntercept(std::string teammate, std::string contact);
   void buildCostMap();
   std::string determineBestIntruderToPursue(); 
   std::string getContactClosestToFlag();
   std::string getTeammateClosestToNearLoiter();

   //std::string buildInterceptMissionTask(std::string target_name); 

 protected:
   void registerVariables();

 private: // Configuration variables
   std::string m_team_color;
   unsigned int m_team_size;
   std::set<std::string> m_opposing_team_names;
   std::set<std::string> m_own_team_names;

   XYPolygon m_home_zone_poly;
   XYPoint m_own_flag_pt;

   // opinion dynamics input function
   double m_dist_to_own_flag_gain;
   double m_enemy_agent_in_home_zone_gain;
   double m_defend_bias;

 private: // State variables
   double m_nav_x;
   double m_nav_y;
   double m_nav_hdg;
   std::string m_vname;

   // Multi-agent book keeping
   std::map<std::string, NodeRecord> m_node_rec_map;
   std::set<std::string> m_contacts;
   std::set<std::string> m_tagged_vehicles;
   
   bool m_we_grabbed_their_flag;
   bool m_they_grabbed_our_flag;

   std::string m_name_of_our_teammate_who_grabbed_their_flag;
   std::string m_name_of_the_opposing_player_who_grabbed_our_flag;

   // opinion dynamics related
   
   // Map to hold the population state:
   // Key = option name
   // Val = set of names of agents for that option.
   //       some sets could be empty!
   std::map<std::string, std::set<std::string>> m_pop_state_map;
   std::string m_curr_option; 
   
   std::map<std::string, double>  m_opinion_inputs;
   std::set<std::string> m_names_of_enemy_agents_in_our_zone;

   double m_dist_to_own_flag;
   double m_agents_in_our_zone; 
   
  
   // blocking related
   std::string m_name_of_closest_enemy_to_player_with_flag;
   double m_dist_of_closest_enemy_to_player_with_flag;
   double m_best_trail_dist;
   double m_best_trail_angle; 


   // defense allocation related
   //double m_min_reallocate_interval;
   //double m_max_reallocate_interval;
   //double m_wait_to_resend_interval;
   // map of next time this task needs to be reassigned
   //std::map<std::string, double> m_time_to_reassign_intruder_map;

   // map of time this task was sent
   //std::map<std::string, double> m_time_task_sent; 
   //unsigned int m_number_of_tasks_sent;

   //bool m_assigned_to_intercept;
   //std::string m_assigned_contact;

   // defense allocation realted 2.0
   // cost map:  key is teammate name (blue_one)
   //            value is: map where
   //                      key is enemy contact name
   //                      value is cost
   std::map<std::string, std::map<std::string, double> > m_cost_map;

   double m_intercept_heading_penalty; 
   double m_intercept_static_lead_dist;
   double m_intercept_speed_extrap;
   bool   m_use_dynamic_speed_based_lead_dist;
   double m_last_grabber_intercept_trail_range; 

   // loiter related
   XYPoint m_near_loiter_pt;
   XYPoint m_far_loiter_pt;
   double  m_loiter_offset;
   double  m_loiter_offset_angle;
};

#endif 
