/************************************************************/
/*    NAME: Supun Randeni                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ChangeMyName.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ChangeMyName_HEADER
#define ChangeMyName_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class ChangeMyName : public AppCastingMOOSApp
{
 public:
   ChangeMyName();
   ~ChangeMyName();

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
 	struct CONFIG
	    {
	        std::string trigger;
	        std::string in;
	        std::string out;
          bool bool_trigger;
          bool bool_trigger_flag;
	    };

	std::vector<CONFIG> config;
	std::vector<std::string>   in_msg_string;
  std::vector<double>        in_msg_double;
  std::vector<bool>          in_msg_isdouble;
  std::vector<bool>          in_msg_sent;
  std::vector<double> trigger_time;
  std::vector<bool> triggered;
     

 private: // State variables
};

#endif 
