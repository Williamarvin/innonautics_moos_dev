/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: MultiAgentColFilt.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef MultiAgentColFilt_HEADER
#define MultiAgentColFilt_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NodeRecord.h"        // for node record
#include "NodeRecordUtils.h"   // for processing incoming node reports

#include "ForwardSetEst.h"     // for fwd reachable set. 

class MultiAgentColFilt : public AppCastingMOOSApp
{
 public:
   MultiAgentColFilt();
   ~MultiAgentColFilt();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   bool checkReadyToGo();
   

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();
   bool handleNodeReport(std::string msg);
   bool handleContactsList(std::string msg);
   void updateContactFwdReachSets();
   bool checkFwdReachSetIntersects();

 protected:
   void registerVariables();

 private: // Configuration variables
   double m_own_state_stale_thresh;
   double m_stale_node_rec_thresh;
   bool   m_verbose;
   bool   m_post_moos_manual_override;
   bool   m_post_rev_thrust;

   // store these to use when a new contact shows up
   std::string  m_nominal_fwd_poly_spec;
   double       m_nominal_fwd_speed;
   double       m_emergency_rev_thrust_cmd;

   std::string m_vname; 

 private: // State variables

   bool m_deployed;
   bool m_emergency_posted;
   std::string m_emergency_stop_contact_name;

   std::map<double, std::string> m_estop_record;
   
   double m_nav_x;
   double m_nav_y;
   double m_nav_hdg;
   double m_nav_spd;

   double m_nav_x_last_rcv_time;
   double m_nav_y_last_rcv_time;
   double m_nav_hdg_last_rcv_time;
   double m_nav_spd_last_rcv_time;
   
   // Book keeping
   std::map<std::string, NodeRecord> m_node_rec_map;
   std::set<std::string> m_contacts;
   std::set<std::string> m_contacts_with_issues;

   // Forward reachability
   ForwardSetEst m_own_fwd_reach_set;

   std::map<std::string, ForwardSetEst>  m_contacts_fwd_reach_set; 
};

#endif 
