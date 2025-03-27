/*****************************************************************/
/*    NAME: Tyler Paine, Mike Defilippo, Mike Benjamin           */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: VehX.h                                               */
/*    DATE: 25 APRIL 2024                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef VehX_HEADER
#define VehX_HEADER

#include <string>
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "SockNinja.h"              // from lib_socket_utils
#include "MBUtils.h"                // from lib_mbutil
#include "LatLonFormatUtils.h"      // from lib_mbutil
#include "Thruster.h"               


class VehX : public AppCastingMOOSApp
{
public:
  VehX();
  ~VehX();

protected: // Standard public MOOSApp functions
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard protected MOOS App functions
  bool buildReport();
  void registerVariables();

protected: // App Specific functions
  void reportWarningsEvents();
  void sendMessagesToSocket();
  void readMessagesFromSocket();

  bool handleConfigIgnoreMsg(std::string);
  bool handleMsgGPRMC(std::string);

  bool reportBadMessage(std::string msg, std::string reason="");
  bool GeodesySetup();
  void checkForStalenessOrAllStop();
  

private: // Config variables
  double       m_max_rudder;       // MAX_RUDDER
  double       m_max_thrust;       // MAX_THRUST
  std::string  m_drive_mode;       // DRIVE_MODE
  std::string  m_vname;            // Vehicle name
  std::set<std::string> m_ignore_msgs;
  
private: // State variables
  CMOOSGeodesy m_geodesy;
  SockNinja    m_ninja;
  Thruster     m_thrust;

  bool         m_ivp_allstop;
  bool         m_moos_manual_override;

  // Stale Message Detection
  bool         m_stale_check_enabled;
  bool         m_stale_mode;
  double       m_stale_threshold;
  unsigned int m_count_stale;
  double       m_tstamp_des_rudder;
  double       m_tstamp_des_thrust;

  double       m_nav_x;
  double       m_nav_y;
  double       m_nav_hdg;
  double       m_nav_spd;

  unsigned int m_bad_nmea_semantic;

};

#endif 


