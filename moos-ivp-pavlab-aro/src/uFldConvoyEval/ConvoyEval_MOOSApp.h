/************************************************************/
/*    NAME: Tyler Paine, Michael R. Benjamin                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ConvoyEval_MOOSApp.h                            */
/*    DATE: June 5th, 2022                                  */
/************************************************************/

#ifndef CONVOY_EVAL_MOOSApp_HEADER
#define CONVOY_EVAL_MOOSApp_HEADER

#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "FldEvalConvoyEngine.h"

class ConvoyEval_MOOSApp : public AppCastingMOOSApp
{
 public:
  ConvoyEval_MOOSApp();
  ~ConvoyEval_MOOSApp() {};

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
  std::set<std::string> m_vehicle_names; // to hold the names of all vehicles 

 private: // State variables
  FldEvalConvoyEngine m_fld_engine;

  std::string m_prev_convoy_order;
  int         m_prev_max_pairwise_range;
  int         m_ever_max_pairwise_range;
};

#endif 
