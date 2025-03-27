/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PopEval.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef PopEval_HEADER
#define PopEval_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <limits>              // for std::numeric_limits<double>::max() and min;

class PopEval : public AppCastingMOOSApp
{
 public:
   PopEval();
   ~PopEval();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();
   
   bool handledNodeStatusMsg(std::string msg);
   void calculateStats();

 protected:
   void registerVariables();

 private: // Configuration variables
   double m_stale_thresh; 

 private: // State variables

   std::map<std::string, double> m_odom_vals;
   std::map<std::string, double> m_batt_vals;
   std::map<std::string, double> m_times;
   std::map<std::string, std::string>  m_raw_msgs;

   std::string m_odom_stat_report;
   std::string m_batt_stat_report;
   
};

#endif 
