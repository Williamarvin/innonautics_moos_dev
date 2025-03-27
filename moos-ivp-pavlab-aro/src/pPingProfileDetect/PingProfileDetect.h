/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PingProfileDetect.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef PingProfileDetect_HEADER
#define PingProfileDetect_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class PingProfileDetect : public AppCastingMOOSApp
{
 public:
   PingProfileDetect();
   ~PingProfileDetect();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();
   bool getIndex(std::vector<int> x);
 protected:
   void registerVariables();
   std::vector< std::vector<int> > m_ping_profiles;
   std::string m_val;
   bool m_get_profile;
   bool m_got_profile;
   int m_start_ind;
   int m_end_ind;
   int m_last_indice;
   std::vector<int> m_total;
 private: // Configuration variables

 private: // State variables
};

#endif 
