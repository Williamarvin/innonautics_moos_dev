/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionAnalysis.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef OpinionAnalysis_HEADER
#define OpinionAnalysis_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class OpinionAnalysis : public AppCastingMOOSApp
{
 public:
   OpinionAnalysis();
   ~OpinionAnalysis();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();
   void handleOption(std::string val);

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
   std::map<std::string, double> m_option_times;
   double m_last_time_option_rec;
   
   std::string m_last_option_name;
   bool m_got_first_option; 
};

#endif 
