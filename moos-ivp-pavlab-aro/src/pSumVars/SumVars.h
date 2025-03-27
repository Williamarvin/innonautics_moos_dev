/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SumVars.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef SumVars_HEADER
#define SumVars_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class SumVars : public AppCastingMOOSApp
{
 public:
   SumVars();
   ~SumVars();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();
   bool handleAlgebraSpec(std::string spec, std::string operation);

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
   std::map<std::string, std::set<std::string> > m_algebra_map;
   std::map<std::string, double> m_vals_map;

   std::map<std::string, bool> m_got_vals_map;

   std::map<std::string,std::string> m_operation_map; 
   // 
};

#endif 
