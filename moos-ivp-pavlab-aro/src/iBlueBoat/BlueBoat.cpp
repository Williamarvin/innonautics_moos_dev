/*****************************************************************/
/*    NAME: Jeremy Wenger, Tyler Paine,  Mike Benjamin           */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BlueBoat.cpp                                         */
/*    DATE: 25 APRIL 2024                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include "BlueBoat.h"
#include <iostream>
#include <unistd.h>

using namespace std;

//---------------------------------------------------------
// Constructor()

BlueBoat::BlueBoat()
{
  // Configuration variables  (overwritten by .moos params)
  m_max_rudder = 30.0;     // default MAX_RUDDER (+/-)
  m_max_thrust = 100.0;    // default MAX_THRUST (+/-)
  m_drive_mode = "normal"; // default DRIVE_MODE ("normal"|"aggro"|"direct")

  m_ivp_allstop = true;
  m_moos_manual_override = true;

  // Stale Message Detection
  m_stale_check_enabled = false;
  m_stale_mode = false;
  m_stale_threshold = 1.5;
  m_count_stale = 0;
  m_tstamp_des_rudder = 0;
  m_tstamp_des_thrust = 0;

  m_bad_nmea_semantic = 0;

  m_nav_x = -1;
  m_nav_y = -1;
  m_nav_hdg = -1;
  m_nav_spd = -1;
  m_nav_alt = -1;
  m_nav_lat = -1;
  m_nav_long = -1;
  m_nav_vx = -1;
  m_nav_vy = -1;
  m_nav_vz = -1;
  m_nav_surge = -1;
  m_nav_sway = -1;
  m_nav_heave = -1;
  m_nav_hdg_rad = -1;

  m_right_rc = -1;
  m_left_rc = -1;
  m_aux_rc = -1;
  m_rssi = -1;

  double m_attitude_pitch = -1;
  double m_attitude_roll = -1;
  double m_attitude_yaw = -1;
  double m_attitude_pitchrate = -1;
  double m_attitude_rollrate = -1;
  double m_attitude_yawrate = -1;

  // Desired Data from Blue Boat should at some point be set in the moos file
  m_desired_messages.push_back("RC_CHANNELS");
  m_desired_messages.push_back("HEARTBEAT");
  m_desired_messages.push_back("GLOBAL_POSITION_INT");
  m_desired_messages.push_back("SYS_STATUS");
  m_desired_messages.push_back("ATTITUDE");

  m_bridge.disableWarningBadNMEALen();
  m_bridge.disableWarningBadNMEAForm();
}

//---------------------------------------------------------
// Destructor()

BlueBoat::~BlueBoat()
{
  m_bridge.closeSockFDs();
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool BlueBoat::OnStartUp()
{

  AppCastingMOOSApp::OnStartUp();

  //------------------------------------------------------
  // HANDLE PARAMETERS IN .MOOS FILE ---------------------
  //------------------------------------------------------
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  m_app_name = GetAppName();

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if ((param == "port") && isNumber(value))
    {
      int port = atoi(value.c_str());
      handled = m_bridge.setPortNumber(port);
    }
    else if (param == "ip_addr")
      handled = m_bridge.setIPAddr(value);
    else if (param == "ivp_allstop")
      handled = setBooleanOnString(m_ivp_allstop, value);
    else if (param == "stale_check_enabled")
      handled = setBooleanOnString(m_stale_check_enabled, value);
    else if (param == "comms_type")
      handled = m_bridge.setCommsType(value);
    else if (param == "stale_thresh")
      handled = setPosDoubleOnString(m_stale_threshold, value);
    else if (param == "max_rudder")
      handled = m_thrust.setMaxRudder(value);
    else if (param == "max_thrust")
      handled = m_thrust.setMaxThrust(value);
    else if (param == "drive_mode")
    {
      handled = m_thrust.setDriveMode(value);
      m_drive_mode = value;
    }
    else if (param == "ignore_msg")
      handled = handleConfigIgnoreMsg(value);
    else if (param == "ignore_checksum_errors")
    {
      bool bool_val = false;
      bool ok1 = setBooleanOnString(bool_val, value);
      bool ok2 = m_bridge.setIgnoreCheckSum(bool_val);
      handled = ok1 && ok2;
    }
    else if ((param == "warn_bad_nmea_len") && (tolower(value) == "false"))
    {
      m_bridge.disableWarningBadNMEALen();
      handled = true;
    }
    else if ((param == "warn_bad_nmea_nend") && (tolower(value) == "false"))
    {
      m_bridge.disableWarningBadNMEANend();
      handled = true;
    }
    else if ((param == "warn_bad_nmea_rend") && (tolower(value) == "false"))
    {
      m_bridge.disableWarningBadNMEARend();
      handled = true;
    }
    else if ((param == "warn_bad_nmea_form") && (tolower(value) == "false"))
    {
      m_bridge.disableWarningBadNMEAForm();
      handled = true;
    }
    else if ((param == "warn_bad_nmea_chks") && (tolower(value) == "false"))
    {
      m_bridge.disableWarningBadNMEAChks();
      handled = true;
    }
    else if ((param == "warn_bad_nmea_key") && (tolower(value) == "false"))
    {
      m_bridge.disableWarningBadNMEAKey();
      handled = true;
    }
    else if (param == "debug")
    {
      m_debug = (value == tolower("true")) ? true : false;
      if (m_debug)
      {
        time_t rawtime;
        struct tm *timeinfo;
        memset(m_fname, m_fname_buff_size, '\0');
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char fmt[m_fname_buff_size];
        memset(fmt, m_fname_buff_size, '\0');
        strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                 m_app_name.c_str(), fmt);
      }
      handled = true;
    }

    if (!handled)
    {
      reportUnhandledConfigWarning(orig);
      list<string> warnings = m_thrust.getWarnings();
      while (!warnings.empty())
      {
        reportConfigWarning(warnings.front());
        warnings.pop_front();
      }
    }
  }

  // Set up TCP connection and send desired messages
  m_bridge.setupConnection();
  m_bridge.setDesiredMessages(m_desired_messages);

  // Init Geodesy
  GeodesySetup();

  bool vnameOk = m_MissionReader.GetValue("Community", m_vname);
  if (!vnameOk)
  {
    reportUnhandledConfigWarning("Not able to get vehicle name");
  }

  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool BlueBoat::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void BlueBoat::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("IVPHELM_ALLSTOP", 0);
  Register("DESIRED_THRUST", 0);
  Register("DESIRED_RUDDER", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool BlueBoat::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    double mtime = msg.GetTime();
    string key = msg.GetKey();
    double dval = msg.GetDouble();
    string sval = msg.GetString();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity(); 
    string msrc  = msg.GetSource();
#endif

    if (key == "IVPHELM_ALLSTOP")
    {
      if (msg.IsString())
      {
        m_ivp_allstop = (toupper(sval) != "CLEAR");
      }
      else
      {
        reportRunWarning("Received Invalid Message:" + key);
      }
    }
    else if (key == "MOOS_MANUAL_OVERRIDE")
    {
      bool ok = setBooleanOnString(m_moos_manual_override, sval);
      if (!ok)
        reportRunWarning("Received Invalid Message:" + key);
    }
    else if (key == "DESIRED_RUDDER")
    {
      if ((msg.IsDouble()) && (m_thrust.getDriveMode() != "direct"))
      {
        m_tstamp_des_rudder = mtime;
        m_thrust.setRudder(dval);
      }
      else
      {
        reportRunWarning("Drive mode set to direct but got rudder command, or value is not double: " + key);
      }
    }
    else if ((msg.IsDouble()) && (key == "DESIRED_THRUST"))
    {
      if (m_thrust.getDriveMode() != "direct")
      {
        m_tstamp_des_thrust = mtime;
        m_thrust.setThrust(dval);
      }
      else
      {
        reportRunWarning("Drive mode set to direct but got thruster command, or value is not double: " + key);
      }
    }
    else if (key != "APPCAST_REQ")
    { // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
  }
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool BlueBoat::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Part 1: Check for allstop or staleness
  checkForStalenessOrAllStop();

  // Part 2: Connect if needed, and write/read from socket
  if (m_bridge.getState() != "connected")
    m_bridge.setupConnection();

  if (m_bridge.getState() == "connected")
  {
    sendMessagesToSocket();
    readMessagesFromSocket();
  }

  // Part 3: Get Appcast events from ninja and report them
  reportWarningsEvents();
  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: GeodesySetup()
//   Purpose: Initialize geodesy object with lat/lon origin.
//            Used for LatLon2LocalUTM conversion.

bool BlueBoat::GeodesySetup()
{
  double LatOrigin = 0.0;
  double LonOrigin = 0.0;

  // Get Latitude Origin from .MOOS Mission File
  bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
  if (!latOK)
  {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return (false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
  if (!lonOK)
  {
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return (false);
  }

  // Initialise CMOOSGeodesy object
  bool geoOK = m_geodesy.Initialise(LatOrigin, LonOrigin);
  if (!geoOK)
  {
    reportConfigWarning("CMOOSGeodesy::Initialise() failed. Invalid origin.");
    return (false);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: sendMessagesToSocket()

void BlueBoat::sendMessagesToSocket()
{
  // This code is still a work in progress the thruster values still need to be
  //  mapped to a pwm value and the auxpwm pin needs to be handled by moos
  //------------------------------------------------
  //  Example for setting thrusters directly
  double thrustL, thrustR;
  double auxpwm = 0.0;

  m_thrust.setDriveMode(m_drive_mode);

  // Calculate the right and left thruster values
  // with whatever DriveMode is selected.
  m_thrust.calcDiffThrust();
  // Update differential thrust values
  thrustL = m_thrust.getThrustLeft();
  thrustR = m_thrust.getThrustRight();

  // convert differential thrust values to usable values for blue boat
  thrustL = 1500.0 + 4 * thrustL;
  thrustR = 1500.0 + 4 * thrustR;

  // This code is modified correcty, the boat expects 3 pwm values

  m_bridge.sendFormattedMessage(thrustL, thrustR, auxpwm);
  sleep(0.5);

  // Publish command to MOOSDB for logging/debugging
  Notify("BBTMS_THRUST_L", thrustL);
  Notify("BBTMS_THRUST_R", thrustR);

  //-------------------------------------------------
}

//---------------------------------------------------------
// Procedure: readMessagesFromSocket()
//      Note: Messages returned from the SockNinja have been
//            confirmed to be valid NMEA format and checksum

void BlueBoat::readMessagesFromSocket()
{
  list<string> incoming_msgs = m_bridge.getSockMessages();
  list<string>::iterator p;
  for (p = incoming_msgs.begin(); p != incoming_msgs.end(); p++)
  {
    string msg = *p;
    msg = biteString(msg, '\r'); // Remove CRLF
    Notify("IBlueBoat_RAW_NMEA", msg);

    // logic tree modified to handle some nmea strings from the blue boat
    bool handled = false;
    if (m_ignore_msgs.count(msg.substr(0, 6)) >= 1)
      handled = true;
    else if (strBegins(msg, "$GBPOS"))
      handled = handleMsgGBPOS(msg);
    else if (strBegins(msg, "$ATITD"))
      handled = handleMsgATITD(msg);
    else if (strBegins(msg, "$BATRY"))
      handled = handleMsgBATRY(msg);
    else if (strBegins(msg, "$RCHNL"))
      handled = handleMsgRCHNL(msg);
    else if (strBegins(msg, "$HRTBT"))
      handled = handleMsgHRTBT(msg);
    else if (strBegins(msg, "$SYSST"))
      handled = handleMsgSYSST(msg);
    else
      reportBadMessage(msg, "Unknown NMEA Key");

    if (!handled)
      m_bad_nmea_semantic++;
  }
}

//---------------------------------------------------------
// Procedure: handleConfigIgnoreMsg()
//  Examples: ignore_msg = $GPGLL
//            ignore_msg = $GPGLL, GPGSV, $GPVTG

bool BlueBoat::handleConfigIgnoreMsg(string str)
{
  bool all_ok = true;

  vector<string> msgs = parseString(str, ',');
  for (unsigned int i = 0; i < msgs.size(); i++)
  {
    string msg = stripBlankEnds(msgs[i]);
    // Check if proper NMEA Header
    if ((msg.length() == 6) && (msg.at(0) = '$'))
      m_ignore_msgs.insert(msg);
    else
      all_ok = false;
  }

  return (all_ok);
}

//---------------------------------------------------------
// Procedure: dbg_print()
bool BlueBoat::dbg_print(const char *format, ...)
{
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_fname, "a");
    if (m_debug_stream != nullptr)
    {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      return true;
    }
    else
    {
      reportRunWarning("Debug mode is enabled and file could not be opened\n");
      return false;
    }
  }
  return false;
}

//---------------------------------------------------------
// Procedure: checkForStalenessOrAllStop()
//   Purpose: If DESIRED_RUDDER or _THRUST commands are stale,
//            set local desired_rudder/thrust to zero.
//            If an all-stop has been posted, also set the
//            local desired_rudder/thrust vals to zero.

void BlueBoat::checkForStalenessOrAllStop()
{
  if (m_ivp_allstop)
  {
    m_thrust.setRudder(0);
    m_thrust.setThrust(0);
    if (m_thrust.getDriveMode() == "direct")
    {
      m_thrust.setThrusterSpeed(0.0, "right");
      m_thrust.setThrusterSpeed(0.0, "left");
    }
    return;
  }

  // If not checking staleness, ensure stale mode false, return.
  if (!m_stale_check_enabled)
  {
    m_stale_mode = false;
    return;
  }

  double lag_rudder = m_curr_time - m_tstamp_des_rudder;
  double lag_thrust = m_curr_time - m_tstamp_des_thrust;

  bool stale_rudder = (lag_rudder > m_stale_threshold);
  bool stale_thrust = (lag_thrust > m_stale_threshold);

  if (stale_rudder)
    m_count_stale++;
  if (stale_thrust)
    m_count_stale++;

  bool stale_mode = false;
  if (stale_rudder || stale_thrust)
  {
    m_thrust.setRudder(0);
    m_thrust.setThrust(0);
    stale_mode = true;
  }

  // Check new stale_mode represents a change from previous
  if (stale_mode && !m_stale_mode)
    reportRunWarning("Stale Command Detected: Stopping Vehicle");
  if (!stale_mode && m_stale_mode)
    retractRunWarning("Stale Command Detected: Stopping Vehicle");

  m_stale_mode = stale_mode;
}

// handle messages

// Example message included for reference
//---------------------------------------------------------
// Procedure: handleMsgBATRY()
//      Note: Not implemented at the moment as SYSST provides voltage and current
//   Example:
//   $BATRY,battery_function:type:MAV_BATTERY_FUNCTION_UNKNOWN,battery_remaining:-1,charge_state:type:MAV_BATTERY_CHARGE_STATE_OK,
//    current_battery:-88,current_consumed:-35,energy_consumed:-20,id:0,mavtype:type:MAV_BATTERY_TYPE_UNKNOWN,temperature:32767,time_remaining:0,
//    voltages:[16531,65535,65535,65535,65535,65535,65535,65535,65535,65535],counter:457,first_update:2024-08-01T14:39:43.857506859Z,
//    frequency:3.0264899730682373,last_update:2024-08-01T14:42:15.365952983Z*1E

bool BlueBoat::handleMsgBATRY(string msg)
{
  return (true);
}

// Example message included for reference
//---------------------------------------------------------
// Procedure: handleMsgSYSST()
//      Note:
//   Example:
//   $SYSST,battery_remaining:-1,current_battery:-84,drop_rate_comm:0,errors_comm:0,errors_count1:0,
//    errors_count2:0,errors_count3:0,errors_count4:0,load:45,onboard_control_sensors_enabled:bits:304120111,
//    onboard_control_sensors_health:bits:51413295,onboard_control_sensors_present:bits:321969455,voltage_battery:16570,
//    counter:399,first_update:2024-08-01T14:39:44.242840167Z,frequency:2.0151515007019043,last_update:2024-08-01T14:43:02.741769602Z*1A
bool BlueBoat::handleMsgSYSST(string msg)
{
  if (!strBegins(msg, "$SYSST,"))
    return (false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  int value_int = 0;
  bool got_int = false;

  vector<string> flds = parseString(msg, ',');
  for (unsigned int i = 0; i < flds.size(); i++)
  {
    std::string fld = flds[i];
    string param = tolower(biteStringX(fld, ':'));
    string value = fld;

    got_int = setIntOnString(value_int, value);

    if (param == "voltage_battery")
    {
      if (!got_int)
        return (false);
      m_sysst_voltage = value_int / 1000.0;
      Notify("STAT_BAT_VOLTAGE", m_sysst_voltage);
    }
    else if (param == "current_battery")
    {
      if (!got_int)
        return (false);
      m_sysst_current = value_int / 100.0;
      Notify("STAT_BAT_CURRENT", m_sysst_current);
    }
  }
  return (true);
}

// Example message included for reference
//---------------------------------------------------------
// Procedure: handleMsgGBPOS()
//      Note:
//   Example:
//   $GBPOS,alt:730,hdg:16560,lat:0,lon:0,relative_alt:739,time_boot_ms:201855,vx:0,vy:0,vz:0,
//    counter:600,first_update:2024-08-01T14:39:43.842028118Z,frequency:3.015075445175171,
//    last_update:2024-08-01T14:43:02.981550392Z*0D

bool BlueBoat::handleMsgGBPOS(string msg)
{
  if (!strBegins(msg, "$GBPOS,"))
    return (false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  int value_int = 0;
  bool got_int = false;

  vector<string> flds = parseString(msg, ',');
  for (unsigned int i = 0; i < flds.size(); i++)
  {
    std::string fld = flds[i];
    string param = tolower(biteStringX(fld, ':'));
    string value = fld;

    got_int = setIntOnString(value_int, value);

    if (param == "alt")
    {
      if (!got_int)
        return (false);
      m_nav_alt = value_int;
      m_nav_alt = m_nav_alt / 1000.0; // millimeters to meters
      Notify("NAV_ALT", m_nav_alt);
    }
    else if (param == "hdg")
    {
      if (!isNumber(value))
        return (false);
      m_nav_hdg = value_int;
      m_nav_hdg = m_nav_hdg / 100.0; // centidegrees to degrees
      Notify("NAV_HEADING", m_nav_hdg);
    }
    else if (param == "lat")
    {
      if (!isNumber(value))
        return (false);
      m_nav_lat = value_int;
      m_nav_lat = m_nav_lat / 10000000.0; // into standard degree format
      Notify("NAV_LAT", m_nav_lat);
    }
    else if (param == "lon")
    {
      if (!isNumber(value))
        return (false);
      m_nav_long = value_int;
      m_nav_long = m_nav_long / 10000000.0; // into standard degree format
      Notify("NAV_LON", m_nav_long);
    }
    else if (param == "vx")
    {
      if (!isNumber(value))
        return (false);
      m_nav_vx = value_int;
      m_nav_vx = m_nav_vx / 100.0; // cm/s to m/s
      Notify("NAV_VX", m_nav_vx);
    }
    else if (param == "vy")
    {
      if (!isNumber(value))
        return (false);
      m_nav_vy = value_int;
      m_nav_vy = m_nav_vy / 100.0; // cm/s to m/s
      Notify("NAV_VY", m_nav_vy);
    }
    else if (param == "vz")
    {
      if (!isNumber(value))
        return (false);
      m_nav_vz = value_int;
      m_nav_vz = m_nav_vz / 100.0; // cm/s to m/s
      Notify("NAV_VZ", m_nav_vz);
    }

    // convert lat an long to local frame:
    double x, y;
    bool ok = m_geodesy.LatLong2LocalGrid(m_nav_lat, m_nav_long, y, x);
    if (ok)
    {
      m_nav_x = x;
      m_nav_y = y;
      Notify("NAV_X", m_nav_x);
      Notify("NAV_Y", m_nav_y);

      // publish speed in m/s decomposed into surge and sway
      m_nav_hdg_rad = M_PI * m_nav_hdg / 180.0;

      m_nav_surge = m_nav_vx * cos(m_nav_hdg_rad) + m_nav_vy * sin(m_nav_hdg_rad);
      m_nav_sway = -m_nav_vx * sin(m_nav_hdg_rad) + m_nav_vy * cos(m_nav_hdg_rad);
      m_nav_heave = m_nav_vz;

      Notify("NAV_SURGE", m_nav_surge);
      Notify("NAV_SWAY", m_nav_sway);
      Notify("NAV_HEAVE", m_nav_heave);

      Notify("NAV_SPEED", m_nav_surge);
    }
  }

  dbg_print("LAT: %.7f, LON: %.7f, NAV_X: %.2f, NAV_Y %.2f, NAV_SPEED: %.2f, HEADING: %.2f \n", m_nav_lat, m_nav_long, m_nav_x, m_nav_y, m_nav_surge, m_nav_hdg);
  return (true);
}
// Example message included for reference
//---------------------------------------------------------
// Procedure: handleMsgHRTBT()
//      Note:
//   Example:
//   $HRTBT,autopilot:type:MAV_AUTOPILOT_ARDUPILOTMEGA,base_mode:bits:65,custom_mode:0,mavlink_version:3,
//    mavtype:type:MAV_TYPE_SURFACE_BOAT,system_status:type:MAV_STATE_ACTIVE,counter:205,first_update:2024-08-01T14:39:42.222191531Z,
//    frequency:1.024999976158142,last_update:2024-08-01T14:43:02.241917468Z*23
bool BlueBoat::handleMsgHRTBT(string msg)
{
  if (!strBegins(msg, "$HRTBT,"))
    return (false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  int value_int = 0;
  bool got_int = false;

  vector<string> flds = parseString(msg, ',');
  for (unsigned int i = 0; i < flds.size(); i++)
  {
    std::string fld = flds[i];
    string param = tolower(biteStringX(fld, ':'));
    string value = fld;

    got_int = setIntOnString(value_int, value);

    if (param == "counter")
    {
      if (!got_int)
        return (false);

      m_hrtbt_counter = value_int;
      Notify("STAT_HRT_CNT", m_hrtbt_counter);
    }
  }
  return (true);
}
// Example message included for reference
//---------------------------------------------------------
// Procedure: handleMsgRCHNL()
//      Note:
//   Example:
//   $RCHNL,chan10_raw:1005,chan11_raw:1542,chan12_raw:1542,chan13_raw:1515,chan14_raw:1510,chan15_raw:1515,chan16_raw:1510,
//   chan17_raw:0,chan18_raw:0,chan1_raw:1500,chan2_raw:1915,chan3_raw:1500,chan4_raw:1502,chan5_raw:1000,
//   chan6_raw:1000,chan7_raw:1000,chan8_raw:1000,chan9_raw:1000,chancount:16,rssi:255,time_boot_ms:201615,
//   counter:399,first_update:2024-08-01T14:39:44.246898147Z,frequency:2.0151515007019043,last_update:2024-08-01T14:43:02.742506413Z*6C

bool BlueBoat::handleMsgRCHNL(string msg)
{
  if (!strBegins(msg, "$RCHNL,"))
    return (false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  int value_int = 0;
  bool got_int = false;

  vector<string> flds = parseString(msg, ',');
  for (unsigned int i = 0; i < flds.size(); i++)
  {
    std::string fld = flds[i];
    string param = tolower(biteStringX(fld, ':'));
    string value = fld;

    got_int = setIntOnString(value_int, value);
    if (param == "chan14_raw")
    {
      if (!got_int)
        return (false);
      m_left_rc = value_int; // microseconds
      Notify("RC_LEFT", m_left_rc);
    }
    else if (param == "chan15_raw")
    {
      if (!got_int)
        return (false);
      m_aux_rc = value_int; // microseconds
      Notify("RC_AUX", m_aux_rc);
    }
    else if (param == "chan16_raw")
    {
      if (!got_int)
        return (false);
      m_right_rc = value_int; // microseconds
      Notify("RC_RIGHT", m_right_rc);
    }
    else if (param == "rssi")
    {
      if (!got_int)
        return (false);
      m_rssi = value_int; // 0-255
      Notify("RC_RSSI", m_rssi);
    }
  }

  return (true);
}
// Example message included for reference
//---------------------------------------------------------
// Procedure: handleMsgATITD()
//      Note:
//   Example:
//    $ATITD,pitch:0.008361952379345894,pitchspeed:-3.978051245212555e-05,roll:-0.0179135762155056,rollspeed:-0.00018928153440356255,
//    time_boot_ms:201855,yaw:2.890336751937866,yawspeed:4.8820627853274345e-05,counter:1995,first_update:2024-08-01T14:39:43.352511563Z,
//    frequency:10.025125503540039,last_update:2024-08-01T14:43:02.991506412Z*7E
bool BlueBoat::handleMsgATITD(string msg)
{
  if (!strBegins(msg, "$ATITD,"))
    return (false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  double value_double = 0;
  bool got_double = false;

  vector<string> flds = parseString(msg, ',');
  for (unsigned int i = 0; i < flds.size(); i++)
  {
    std::string fld = flds[i];
    string param = tolower(biteStringX(fld, ':'));
    string value = fld;

    got_double = setDoubleOnString(value_double, value);

    if (param == "pitch")
    {
      if (!got_double)
        return (false);
      m_attitude_pitch = value_double; // radians
      Notify("ATTITUDE_PITCH", m_attitude_pitch);
    }
    else if (param == "roll")
    {
      if (!got_double)
        return (false);
      m_attitude_roll = value_double; // radians
      Notify("ATTITUDE_ROLL", m_attitude_roll);
    }
    else if (param == "yaw")
    {
      if (!got_double)
        return (false);
      m_attitude_yaw = value_double; // radians
      Notify("ATTITUDE_YAW", m_attitude_yaw);
    }
    else if (param == "pitchspeed")
    {
      if (!got_double)
        return (false);
      m_attitude_pitchrate = value_double; // radians/s
      Notify("ATTITUDE_PITCHSPEED", m_attitude_pitchrate);
    }
    else if (param == "rollspeed")
    {
      if (!got_double)
        return (false);
      m_attitude_rollrate = value_double; // radians/s
      Notify("ATTITUDE_ROLLSPEED", m_attitude_rollrate);
    }
    else if (param == "yawspeed")
    {
      if (!got_double)
        return (false);
      m_attitude_yawrate = value_double; // radians/s
      Notify("ATTITUDE_YAWSPEED", m_attitude_yawrate);
    }
  }

  dbg_print("Pitch: %.2f, Roll: %.2f \n", m_attitude_pitch, m_attitude_roll);
  return (true);
}
//---------------------------------------------------------
// Procedure: reportBadMessage()

bool BlueBoat::reportBadMessage(string msg, string reason)
{
  reportRunWarning("Bad NMEA Msg: " + reason + ": " + msg);
  Notify("IBlueBoat_BAD_NMEA", reason + ": " + msg);
  return (false);
}

//---------------------------------------------------------
// Procedure: reportWarningsEvents()
//      Note: Get the AppCast-consistent events, warnings and
//            retractions from the sock ninja for posting

void BlueBoat::reportWarningsEvents()
{
  // Part 1: Handle Event Messages()
  list<string> events = m_bridge.getEvents();
  list<string>::iterator p;
  for (p = events.begin(); p != events.end(); p++)
  {
    string event_str = *p;
    reportEvent(event_str);
  }

  // Part 2: Handle Warning Messages()
  list<string> warnings = m_bridge.getWarnings();
  list<string> thrust_warnings = m_thrust.getWarnings();
  warnings.splice(warnings.end(), thrust_warnings);
  for (p = warnings.begin(); p != warnings.end(); p++)
  {
    string warning_str = *p;
    reportRunWarning(warning_str);
  }

  // Part 3: Handle Retraction Messages()
  list<string> retractions = m_bridge.getRetractions();
  for (p = retractions.begin(); p != retractions.end(); p++)
  {
    string retraction_str = *p;
    retractRunWarning(retraction_str);
  }
}

//------------------------------------------------------------
// Procedure: buildReport()
//
// -------------------------------------------
// Config:   max_r/t: 30/100      stale_check:  false
//           dr_mode: normal      stale_thresh: 15
// -------------------------------------------
// Drive     des_rud: -30         des_thrust_L: 0
// State:    des_thr: 40          des_thrust_R: 0
// -------------------------------------------
// Nav:      nav_x: 5968          nav_hdg: 0
//           nav_y: -6616.3       nav_spd: 0.5
// -------------------------------------------
// Comms:    Type: client         IPv4: 127.0.0.1 (of server)
//           Format: nmea         Port: 29500
//           Status: connected
// ---------------------------
// NMEA sentences:
// <--R     230  $CPNVG,105707.24,0000.00,N,00000.00,W,1,,,0,,,105707.24*64
// <--R     230  $CPRBS,105707.24,1,15.2,15.1,15.3,0*67
// <--R     230  $GPRMC,105707.24,A,0000.00,N,00000.00,W,1.1663,0,291263,0,E*76
//  S-->    231  $PYDIR,0,0*56

bool BlueBoat::buildReport()
{
  string str_max_rud = doubleToStringX(m_max_rudder, 1);
  string str_max_thr = doubleToStringX(m_max_thrust, 1);
  string str_max_both = str_max_rud + "/" + str_max_thr;
  string str_des_rud = doubleToStringX(m_thrust.getRudder(), 1);
  string str_des_thr = doubleToStringX(m_thrust.getThrust(), 1);
  string str_des_thrL = doubleToStringX(m_thrust.getThrustLeft(), 1);
  string str_des_thrR = doubleToStringX(m_thrust.getThrustRight(), 1);

  string str_sta_thr = doubleToStringX(m_stale_threshold, 1);
  string str_sta_ena = boolToString(m_stale_check_enabled);

  string str_nav_x = doubleToStringX(m_nav_x, 1);
  string str_nav_y = doubleToStringX(m_nav_y, 1);
  string str_nav_hdg = doubleToStringX(m_nav_hdg, 1);
  string str_nav_spd = doubleToStringX(m_nav_surge, 1);

  string pd_ruth = padString(str_max_both, 10, false);
  string pd_drmo = padString(m_drive_mode, 10, false);
  string pd_drud = padString(str_des_rud, 10, false);
  string pd_dthr = padString(str_des_thr, 10, false);
  string pd_navx = padString(str_nav_x, 10, false);
  string pd_navy = padString(str_nav_y, 10, false);

  m_msgs << "Config:    max_r/t: " << pd_ruth << "   stale_check:  " << str_sta_ena << endl;
  m_msgs << "           dr_mode: " << pd_drmo << "   stale_thresh: " << str_sta_thr << endl;
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "Drive:     des_rud: " << pd_drud << "   des_thrust_L: " << str_des_thrL << endl;
  m_msgs << "State:     des_thr: " << pd_dthr << "   des_thrust_R: " << str_des_thrR << endl;
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "Nav:       nav_x:   " << pd_navx << "   nav_hdg: " << str_nav_hdg << endl;
  m_msgs << "           nav_y:   " << pd_navy << "   nav_spd: " << str_nav_spd << endl;
  m_msgs << "------------------------------------------------------" << endl;

  list<string> summary_lines = m_bridge.getSummary();
  list<string>::iterator p;
  for (p = summary_lines.begin(); p != summary_lines.end(); p++)
  {
    string line = *p;
    m_msgs << line << endl;
  }

  return (true);
}
