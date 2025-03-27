/*****************************************************************/
/*    NAME: Jeremy Wenger, Tyler Paine,  Mike Benjamin           */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BlueBoat.h                                           */
/*    DATE: 25 APRIL 2024                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef BlueBoat_HEADER
#define BlueBoat_HEADER

#include <string>
#include <cmath>
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "SockNinja.h"         // from lib_socket_utils
#include "MBUtils.h"           // from lib_mbutil
#include "LatLonFormatUtils.h" // from lib_mbutil
#include "Thruster.h"
#include "BlueBoatBridge.hpp"

class BlueBoat : public AppCastingMOOSApp
{
public:
  BlueBoat();
  ~BlueBoat();

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
  bool handleMsgBATRY(std::string);
  bool handleMsgGBPOS(std::string);
  bool handleMsgHRTBT(std::string);
  bool handleMsgRCHNL(std::string);
  bool handleMsgATITD(std::string);
  bool handleMsgSYSST(std::string);
  bool dbg_print(const char *format, ...);

  bool reportBadMessage(std::string msg, std::string reason = "");
  bool GeodesySetup();
  void checkForStalenessOrAllStop();

private:                    // Config variables
  double m_max_rudder;      // MAX_RUDDER
  double m_max_thrust;      // MAX_THRUST
  std::string m_drive_mode; // DRIVE_MODE
  std::string m_vname;      // Vehicle name
  std::set<std::string> m_ignore_msgs;

private: // State variables
  CMOOSGeodesy m_geodesy;
  BlueBoatBridge m_bridge;
  Thruster m_thrust;

  bool m_ivp_allstop;
  bool m_moos_manual_override;

  // Stale Message Detection
  bool m_stale_check_enabled;
  bool m_stale_mode;
  double m_stale_threshold;
  unsigned int m_count_stale;
  double m_tstamp_des_rudder;
  double m_tstamp_des_thrust;

  double m_nav_x;
  double m_nav_y;
  double m_nav_hdg;
  double m_nav_hdg_rad;
  double m_nav_spd;
  double m_nav_alt;
  double m_nav_lat;
  double m_nav_long;
  double m_nav_vx;
  double m_nav_vy;
  double m_nav_vz;
  double m_nav_surge;
  double m_nav_sway;
  double m_nav_heave;

  double m_right_rc;
  double m_left_rc;
  double m_aux_rc;
  double m_rssi;

  double m_attitude_pitch;
  double m_attitude_roll;
  double m_attitude_yaw;
  double m_attitude_pitchrate;
  double m_attitude_rollrate;
  double m_attitude_yawrate;

  double m_sysst_voltage;
  double m_sysst_current;

  double m_hrtbt_counter;

  std::string m_app_name;
  bool m_debug;
  char m_fname[80];
  const long unsigned int m_fname_buff_size = sizeof(m_fname);
  FILE *m_debug_stream;

  std::list<std::string> m_desired_messages;

  unsigned int m_bad_nmea_semantic;
};

#endif
