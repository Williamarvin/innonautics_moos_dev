/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: FalconRunMgr.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef FalconRunMgr_HEADER
#define FalconRunMgr_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"


class FalconRunMgr : public AppCastingMOOSApp
{
 public:
   FalconRunMgr();
   ~FalconRunMgr();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool sendFaultMsg();
   bool sendRadChangeMsg();
   bool handleLoopID(std::string val);

 private: // Configuration variables
   std::string m_midpoint_flag_var_name;
   std::string m_loop_id_var_name;
   std::set<std::string> m_fault_msg;
   std::set<std::string> m_rad_change_msg;
   
   
   int m_loops_before_induce_fault;
   int m_loops_before_change_rad;
   int m_loops_before_return;
   double m_time_delay_rad_msg; 

 private: // State variables
   int m_curr_loop_id;
   bool m_sent_fault;
   bool m_sent_rad_change;
   bool m_sent_return;

   double m_time_rad_flag_rcv; 

};

#endif 
