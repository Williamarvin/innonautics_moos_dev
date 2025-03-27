/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionFixer.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef OpinionFixer_HEADER
#define OpinionFixer_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class OpinionFixer : public AppCastingMOOSApp
{
 public:
   OpinionFixer();
   ~OpinionFixer();

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
   std::set<std::string> m_explore_set;
   std::set<std::string> m_exploit_set;

   std::string m_input_var;
   std::string m_output_var;
   std::string m_vname; 

 private: // State variables
};

#endif 
