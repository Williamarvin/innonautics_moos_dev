/************************************************************/
/*    NAME: Filip Stromstad                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: TurnInPlace.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef TurnInPlace_HEADER
#define TurnInPlace_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"


class TurnInPlace : public AppCastingMOOSApp
{
 public:
   TurnInPlace();
   ~TurnInPlace();

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
    XYPoint m_desired_point;
    double m_desired_heading;
    double m_zero;
    
    std::string m_os_name;
    std::string m_mode;

    // Behaviour-like variables
    std::string m_update_variable;
    std::string m_condition_variable;
    std::string m_condition_value;
    bool m_condition_satisfied;
    bool m_completed;
    std::string m_endflag_variable;
    std::string m_endflag_value;

 private: // State variables
    double m_osx;
    double m_osy;
    double m_heading;
    std::string m_heading_variable;
    double m_last_heading_diff;

    double m_cumulative_heading_diff;
    double P_effort;
    double I_effort;
    double D_effort;
};

#endif 
