/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: HeronSim.h                                           */
/*    DATE: March 18th, 2020                                     */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef HERON_SIM_HEADER
#define HERON_SIM_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "SockNinja.h"
#include "TStamp.h"
#include <cmath>  // for isnan()

#define WATCHDOG_TIME  1.0
#define LOWEST_BATT_V  13.4

class HeronSim : public AppCastingMOOSApp
{
public:
  HeronSim();
  ~HeronSim() {};
  
  bool    Iterate();
  bool    OnNewMail(MOOSMSG_LIST &NewMail);
  bool    OnConnectToServer();
  bool    OnStartUp();

protected:
  bool    ReadIncoming();
  void    PublishToClient(std::string str);
  bool    DealWithNMEA(std::string nmea);
  
  bool    PublishCPNVG();
  bool    PublishCPRBS();
  bool    PublishGPRMC();
  bool    PublishCPNVR();
  bool    PublishCPRCM(); 

  bool    ReceivePYDIR(std::string nmea);
  void    publishThrustLR(double left, double right);

  void    checkForStalePYDIR();
  void    reportWarningsEvents();
  bool    ParseCurrentTime();
  bool    buildReport();

protected:
  void    RegisterVariables();
  bool    isValidNMEA(std::string);
  void    updateHeadingRate(double time); 
  
private: // State variables
  double  m_curr_spd;
  double  m_curr_hdg;
  double  m_curr_lat;
  double  m_curr_lon;
  double  m_curr_hdg_rate;
  double  m_mag_declination_deg; 

  double  m_last_hdg;
  double  m_last_hdg_time; 

  double  m_cur_thrust_lft;
  double  m_cur_thrust_rgt;

  double  m_CPNVGlastTime;
  double  m_CPRBSlastTime;
  double  m_GPRMClastTime;
  double  m_CPNVRlastTime;
  double  m_CPRCMlastTime;
  
  TStamp  m_utc_now;
  
  double  m_LastValidPYDIRtime;

  SockNinja m_ninja;
};

#endif


