/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GroupComboAlloc.h                                    */
/*    DATE: July 2024                                       */
/************************************************************/

#ifndef GroupComboAlloc_HEADER
#define GroupComboAlloc_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include <set>
#include "NodeRecord.h"        // for node record
#include "NodeRecordUtils.h"   // for processing incoming node reports
#include "GeomUtils.h"         // for distPointToPoint
#include "Hungarian.h"         // for HungarianAlgorithm to solve assignments
#include "XYPoint.h"           // for waypoint updating
#include "AngleUtils.h"        // for relAng
#include "XYFormatUtilsPoint.h" // for string2Point




class GroupComboAlloc : public AppCastingMOOSApp
{
 public:
   GroupComboAlloc();
   ~GroupComboAlloc();

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
   bool handleInterceptEstimate(std::string msg);
   bool handleSampleLoc(std::string msg);
   bool handleExtraAgentCost(std::string msg);
   

   void parseInterceptEstimate(std::string spec, std::string &name, double &x, double &y);


   void determineBestTargetToIntercept(bool include_own);
   void updateBehaviors();

   // intercept cost related
   int    getTotalTeamSize(bool include_own);
   void   buildCostMap(bool include_own, bool rebuild, std::set<std::string> allocated_samples);
   double getCostToIntercept(std::string teammate, double intrd_x, double intrd_y);
  std::map<std::string, double>  getAgentCosts(std::string vname, std::set<std::string> allocated_samples);

   std::vector<std::vector<double>> getDistMatrixFromCostMap();
  void postAssignmentsMap(bool include_own, double total_cost);
 

 protected:
   void registerVariables();

 private: // Configuration variables
   
   bool m_hvu_protect;
   std::string m_hvu_name;
  std::string m_active_option_name;   // ex. Intercept, Exploit

   double m_intercept_heading_penalty;
   double m_assigned_cost;
   double m_unassigned_penalty;
   
   double m_primary_trail_range;
   double m_group_trail_range_offset;
   double m_group_trail_angle_offset;

   bool m_post_behavior_updates;
   bool m_calc_marginal_cost_improvement;
   
  

 private: // State variables
   double m_nav_x;
   double m_nav_y;
   double m_nav_hdg;
   double m_nav_spd;
   std::string m_vname;
   std::string m_curr_option;		

   // Multi-agent book keeping
   std::map<std::string, NodeRecord> m_node_rec_map;
   std::set<std::string> m_contacts;

   std::list<std::string> m_intercept_estimates;
   std::set<std::string> m_intruder_set;
   std::vector<std::string> m_intruder_vec;  // used to maintain order

  // This extra cost is set by an incomming message,
  // so that other processes can add costs
  std::map<std::string,double> m_agent_extra_cost;

  // This extra cost is for when an agent was already assigned
  // in a previous round(s).  It is cleared at the start of
  // each assignement
  std::map<std::string,double> m_agent_already_assigned_cost;
   
   // Map to hold the population state:
   // Key = option name
   // Val = set of names of agents for that option.
   //       some sets could be empty!
   std::map<std::string, std::set<std::string>> m_pop_state_map;
   
   std::set<std::string> m_valid_option_lists;


   ////////////////////////////////////////////
   // intercept allocation
   // cost map:  key is teammate name (blue_one)
   //            value is: map where
   //                      key is enemy contact name
   //                      value is cost
   std::map<std::string, std::map<std::string, double> > m_cost_map;

   std::vector<std::vector<double>> m_DistMatrix;
   HungarianAlgorithm HungAlgo;

   // assignements map
   // assignments_map:  key is the target name
   //                   value is an ordered vector of
   //                   teammates assigned to this target
   //                   where the best teammate to intercept
   //                   (lowest cost) is first.
   
   std::map<std::string, std::vector<std::string>> m_assignments_map; 

   std::vector<std::string> m_unalloc_teammates_vec;  // used to maintain order

  // For own assigments
   std::string m_own_target;
   unsigned int m_own_target_priority;

   
};

#endif 
