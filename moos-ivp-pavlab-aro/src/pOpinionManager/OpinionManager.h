/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionManager.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef OpinionManager_HEADER
#define OpinionManager_HEADER

#include "OpinionManagerEngine.h"
#include "NodeMessage.h"

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class OpinionManager : public AppCastingMOOSApp
{
 public:
   OpinionManager();
   ~OpinionManager();

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
   
   OpManagerEngine m_op_eng;
   NodeMessage m_optn_node_message;
   std::string m_vname;

   // Record of last postings to not bloat the MOOSDB
   // key = var name, val is the string that was last posted
   std::map<std::string, std::string> m_last_population_state_posting;
   std::map<std::string, std::string> m_last_option_active_posting;

   
};

#endif 
