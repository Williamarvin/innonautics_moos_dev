/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BathyChecker.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef BathyChecker_HEADER
#define BathyChecker_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <map>

class BathyChecker : public AppCastingMOOSApp
{
 public:
   BathyChecker();
   ~BathyChecker();

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

 private: // State variables
  std::map<std::string,int> m_completed_names;
};

#endif 
