/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SonarSimDetect.h                                          */
/*    DATE: May 26, 2020                                */
/************************************************************/

#ifndef SonarSimDetect_HEADER
#define SonarSimDetect_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYRangePulse.h"  // for send pulse

class SonarSimDetect : public AppCastingMOOSApp
{
 public:
   SonarSimDetect();
   ~SonarSimDetect();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
 protected:
  bool setTarget(std::string name);
  bool handleNodeReport(std::string node_report);
  bool checkDetect();
  void setSonarRad();
  void sendPulse();
  void postDetectFlags(); 
  bool handleDetectFlagDouble(std::string val);
  
    
 protected: // App variables
  //Target Related Variables
  double m_target_x;
  double m_target_y;
  double m_target_depth;
  double m_target_time;
  //Own Ship Variables
  double m_osx;
  double m_osy;
  double m_os_speed;
  //Sonar Related Variables
  double m_beam_angle;
  double m_sonar_rad;
  double m_hz;
  double m_last_report_time;
  double m_time_l;
  bool m_time_limit;
  //Probability and detection variables
  bool m_gotx;
  bool m_goty;
  double m_sigma;


 private: // Configuration variables
    std::string m_target_name;
    std::map<std::string, double> m_detect_double_flags;

    bool m_active;
 private: // State variables
};

#endif 
