/************************************************************/
/*    NAME: Mikala Molina                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RequestPoints.h                                          */
/*    DATE: June 1 2023                             */
/************************************************************/

#ifndef RequestPoints_HEADER
#define RequestPoints_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
using namespace std;
class RequestPoints : public AppCastingMOOSApp
{
 public:
   RequestPoints();
   ~RequestPoints();

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
string m_vname;
double m_min_swimmers; 

 private: // State variables
 double m_swimmers_remaining;
 bool m_need_swimmer; 
 bool m_ready_to_receive;
};

#endif
