/*************************************************#include "XYFormatUtilsSegl.h"***********/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: AdjPath.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef AdjPath_HEADER
#define AdjPath_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "XYSegList.h"
#include "XYFormatUtilsSegl.h"

class AdjPath : public AppCastingMOOSApp
{
 public:
   AdjPath();
   ~AdjPath();

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
   XYSegList m_seg_list;
   std::string m_output_var;
   std::string m_output_prefix;
   std::string m_initial_spec;
   

 private: // State variables
};

#endif 
