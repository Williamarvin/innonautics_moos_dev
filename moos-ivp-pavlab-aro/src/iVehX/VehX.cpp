/*****************************************************************/
/*    NAME: Tyler Paine, Mike Defilippo, Mike Benjamin           */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: VehX.cpp                                             */
/*    DATE: 25 APRIL 2024                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/


#include "VehX.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

VehX::VehX()
{
  // Configuration variables  (overwritten by .moos params)
  m_max_rudder   = 30.0;        // default MAX_RUDDER (+/-)
  m_max_thrust   = 100.0;       // default MAX_THRUST (+/-)
  m_drive_mode   = "normal";    // default DRIVE_MODE ("normal"|"aggro"|"direct")

  m_ivp_allstop      = true;
  m_moos_manual_override = true;

  // Stale Message Detection
  m_stale_check_enabled = false;
  m_stale_mode          = false;
  m_stale_threshold     = 1.5;
  m_count_stale         = 0;
  m_tstamp_des_rudder   = 0;
  m_tstamp_des_thrust   = 0;

  m_bad_nmea_semantic   = 0;

  m_nav_x   = -1;
  m_nav_y   = -1;
  m_nav_hdg = -1;
  m_nav_spd = -1;

  m_ninja.disableWarningBadNMEALen();
  m_ninja.disableWarningBadNMEAForm();


  
}

//---------------------------------------------------------
// Destructor()

VehX::~VehX()
{
  m_ninja.closeSockFDs();
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool VehX::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  //------------------------------------------------------
  // HANDLE PARAMETERS IN .MOOS FILE ---------------------
  //------------------------------------------------------
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if((param == "port") && isNumber(value)) {
      int port = atoi(value.c_str());
      handled = m_ninja.setPortNumber(port);
    }
    else if(param == "ip_addr")
      handled = m_ninja.setIPAddr(value);
    else if(param == "ivp_allstop")
      handled = setBooleanOnString(m_ivp_allstop, value);
    else if(param == "stale_check_enabled")
      handled = setBooleanOnString(m_stale_check_enabled, value);
    else if(param == "comms_type")
      handled = m_ninja.setCommsType(value);
    else if(param == "stale_thresh")
      handled = setPosDoubleOnString(m_stale_threshold, value);
    else if(param == "max_rudder")
      handled = m_thrust.setMaxRudder(value);
    else if(param == "max_thrust")
      handled = m_thrust.setMaxThrust(value);
    else if(param == "drive_mode"){
      handled = m_thrust.setDriveMode(value);
      m_drive_mode = value;
    }
    else if(param == "ignore_msg") 
      handled = handleConfigIgnoreMsg(value);
    else if(param == "ignore_checksum_errors") {
      bool bool_val = false;
      bool ok1 = setBooleanOnString(bool_val, value);
      bool ok2 = m_ninja.setIgnoreCheckSum(bool_val);
      handled = ok1 && ok2;
    }
    else if((param == "warn_bad_nmea_len") && (tolower(value) == "false")) {
      m_ninja.disableWarningBadNMEALen();
      handled = true;
    }
    else if((param == "warn_bad_nmea_nend") && (tolower(value) == "false")) {
      m_ninja.disableWarningBadNMEANend();
      handled = true;
    }      
    else if((param == "warn_bad_nmea_rend") && (tolower(value) == "false")) {
      m_ninja.disableWarningBadNMEARend();
      handled = true;
    }
    else if((param == "warn_bad_nmea_form") && (tolower(value) == "false")) {
      m_ninja.disableWarningBadNMEAForm();
      handled = true;
    }
    else if((param == "warn_bad_nmea_chks") && (tolower(value) == "false")) {
      m_ninja.disableWarningBadNMEAChks();
      handled = true;
    }
    else if((param == "warn_bad_nmea_key") && (tolower(value) == "false")) {
      m_ninja.disableWarningBadNMEAKey();
      handled = true;
    }
    
    if(!handled){
      reportUnhandledConfigWarning(orig);
      list<string> warnings = m_thrust.getWarnings();
      while (!warnings.empty()){
        reportConfigWarning(warnings.front());
        warnings.pop_front();
      }
    }
  }
  
  // Init Geodesy 
  GeodesySetup();

  bool vnameOk = m_MissionReader.GetValue("Community", m_vname);
  if (!vnameOk) {
    reportUnhandledConfigWarning("Not able to get vehicle name");
  }
  
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool VehX::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void VehX::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("IVPHELM_ALLSTOP", 0);
  Register("DESIRED_THRUST",  0);
  Register("DESIRED_RUDDER",  0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
  
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool VehX::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);
  
  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    double mtime  = msg.GetTime();
    string key    = msg.GetKey();
    double dval   = msg.GetDouble();
    string sval   = msg.GetString();
    
#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity(); 
    string msrc  = msg.GetSource();  
#endif

   
    if(key == "IVPHELM_ALLSTOP") {
      if (msg.IsString()) {
      m_ivp_allstop = (toupper(sval) != "CLEAR");
      } else {
	reportRunWarning("Received Invalid Message:" + key);
      }
      
    } else if(key == "MOOS_MANUAL_OVERRIDE") {
      bool ok = setBooleanOnString(m_moos_manual_override, sval);
      if(!ok)
	reportRunWarning("Received Invalid Message:" + key);
      
    } else if (key == "DESIRED_RUDDER" ){
      if ( (msg.IsDouble()) && (m_thrust.getDriveMode() != "direct") ) {
	m_tstamp_des_rudder = mtime;
	m_thrust.setRudder(dval);	
      } else {
	reportRunWarning("Drive mode set to direct but got rudder command, or value is not double: " + key);
      }
      
    } else if ( (msg.IsDouble()) && (key == "DESIRED_THRUST") ) {
      if ( m_thrust.getDriveMode() != "direct" ) {
	m_tstamp_des_thrust = mtime;
	m_thrust.setThrust(dval);
      } else {
	reportRunWarning("Drive mode set to direct but got thruster command, or value is not double: " + key);
      }
      
    } else if(key != "APPCAST_REQ") {// handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }

  }
  return(true);
}


//---------------------------------------------------------
// Procedure: Iterate()

bool VehX::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  // Part 1: Check for allstop or staleness
  checkForStalenessOrAllStop();
    
  // Part 2: Connect if needed, and write/read from socket
  if(m_ninja.getState() != "connected")
    m_ninja.setupConnection();

  if(m_ninja.getState() == "connected") {
    sendMessagesToSocket();
    readMessagesFromSocket();
  }

  // Part 3: Get Appcast events from ninja and report them
  reportWarningsEvents();
  AppCastingMOOSApp::PostReport();
  return(true);
}


//---------------------------------------------------------
// Procedure: GeodesySetup()
//   Purpose: Initialize geodesy object with lat/lon origin.
//            Used for LatLon2LocalUTM conversion.

bool VehX::GeodesySetup()
{
  double LatOrigin = 0.0;
  double LonOrigin = 0.0;

  // Get Latitude Origin from .MOOS Mission File
  bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
  if(!latOK) {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return(false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
  if(!lonOK){
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return(false);
  }

  // Initialise CMOOSGeodesy object
  bool geoOK = m_geodesy.Initialise(LatOrigin, LonOrigin);
  if(!geoOK) {
    reportConfigWarning("CMOOSGeodesy::Initialise() failed. Invalid origin.");
    return(false);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: sendMessagesToSocket()

void VehX::sendMessagesToSocket()
{

  //------------------------------------------------
  // Example for setting thrusters directly
  double thrustL, thrustR;
  m_thrust.setDriveMode(m_drive_mode);

  // Calculate the right and left thruster values
  // with whatever DriveMode is selected.
  m_thrust.calcDiffThrust();
  // Update differential thrust values
  thrustL = m_thrust.getThrustLeft();
  thrustR = m_thrust.getThrustRight();


  // Send the primary PYDIR front seat command
  string msg = "PYDIR,";
  msg += doubleToStringX(thrustL, 1) + ",";
  msg += doubleToStringX(thrustR, 1);

  msg = "$" + msg + "*" + checksumHexStr(msg) + "\r\n";

  m_ninja.sendSockMessage(msg);

  // Publish command to MOOSDB for logging/debugging
  Notify("PYDIR_THRUST_L", thrustL);
  Notify("PYDIR_THRUST_R", thrustR);

  //-------------------------------------------------

}

//---------------------------------------------------------
// Procedure: readMessagesFromSocket()
//      Note: Messages returned from the SockNinja have been
//            confirmed to be valid NMEA format and checksum

void VehX::readMessagesFromSocket()
{
  list<string> incoming_msgs = m_ninja.getSockMessages();
  list<string>::iterator p;
  for(p=incoming_msgs.begin(); p!=incoming_msgs.end(); p++) {
    string msg = *p;
    msg = biteString(msg, '\r'); // Remove CRLF
    Notify("IVEHX_RAW_NMEA", msg);

    bool handled = false;
    if(m_ignore_msgs.count(msg.substr (0,6)) >= 1) 
      handled = true;
    else if ( strBegins(msg, "$GPRMC") || strBegins(msg, "$GNRMC") )  // example
      handled = handleMsgGPRMC(msg);
    else
      reportBadMessage(msg, "Unknown NMEA Key");
            
    if(!handled)
      m_bad_nmea_semantic++;
  }
}

//---------------------------------------------------------
// Procedure: handleConfigIgnoreMsg()
//  Examples: ignore_msg = $GPGLL
//            ignore_msg = $GPGLL, GPGSV, $GPVTG

bool VehX::handleConfigIgnoreMsg(string str)
{
  bool all_ok = true;
  
  vector<string> msgs = parseString(str, ',');
  for(unsigned int i=0; i<msgs.size(); i++) {
    string msg = stripBlankEnds(msgs[i]);
    // Check if proper NMEA Header
    if((msg.length() == 6) && (msg.at(0) = '$'))
      m_ignore_msgs.insert(msg);
    else
      all_ok = false;
  }

  return(all_ok);
}

//---------------------------------------------------------
// Procedure: checkForStalenessOrAllStop()
//   Purpose: If DESIRED_RUDDER or _THRUST commands are stale,
//            set local desired_rudder/thrust to zero.
//            If an all-stop has been posted, also set the
//            local desired_rudder/thrust vals to zero.

void VehX::checkForStalenessOrAllStop()
{
  if(m_ivp_allstop) {
    m_thrust.setRudder(0);
    m_thrust.setThrust(0);
    if ( m_thrust.getDriveMode() == "direct" ) {
      m_thrust.setThrusterSpeed(0.0, "right");
      m_thrust.setThrusterSpeed(0.0, "left");
    }
    return;
  }

  // If not checking staleness, ensure stale mode false, return.
  if(!m_stale_check_enabled) {
    m_stale_mode = false;
    return;
  }

  double lag_rudder = m_curr_time - m_tstamp_des_rudder;
  double lag_thrust = m_curr_time - m_tstamp_des_thrust;

  bool stale_rudder = (lag_rudder > m_stale_threshold);
  bool stale_thrust = (lag_thrust > m_stale_threshold);

  if(stale_rudder)
    m_count_stale++;
  if(stale_thrust)
    m_count_stale++;

  bool stale_mode = false;
  if(stale_rudder || stale_thrust) {
    m_thrust.setRudder(0);
    m_thrust.setThrust(0);
    stale_mode = true;
  }

  // Check new stale_mode represents a change from previous
  if(stale_mode && !m_stale_mode) 
    reportRunWarning("Stale Command Detected: Stopping Vehicle");
  if(!stale_mode && m_stale_mode) 
    retractRunWarning("Stale Command Detected: Stopping Vehicle");

  m_stale_mode = stale_mode;
}


// Example message included for reference, for more message
// handling functions, see iM300
//---------------------------------------------------------
// Procedure: handleMsgGPRMC()
//      Note: Proper NMEA format and checksum prior confirmed
//            for GPRMC and GNRMC NMEA messages. 
//   Example:
//   $GPRMC,150942.619,A,0000.00,N,00000.00,W,1.1663,0,291263,0,E*41

//  0   $GPRMC
//  1 [Timestamp]    UTC of position fix
//  2 [Data status]  A-ok, V-invalid
//  3 [Lat_NMEA]     Calculated latitude, in NMEA format
//  4 [LatNS_NMEA]   Hemisphere (N or S) of latitude
//  5 [Lon_NMEA]     Calculated longitude, in NMEA format
//  6 [LonEW_NMEA]   Hemisphere (E or W) of longitude
//  7 [Speed]        Speed over ground in Knots
//  8 [Course]       True Course, Track made good in degrees
//  9 [DepthTop]     Date of Fix
// 10 [Mag Var]      Magnetic variation degrees
// 11 [Mag Var E/W]  Easterly subtracts from true course

bool VehX::handleMsgGPRMC(string msg)
{
  if( !( strBegins(msg, "$GPRMC,") || strBegins(msg, "$GNRMC,") ) )
    return(false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 13) {
    if(!m_ninja.getIgnoreCheckSum())
      return(reportBadMessage(msg, "Wrong field count"));
  } 
  
  if((flds[4] != "N") && (flds[4] != "S"))
    return(reportBadMessage(msg, "Bad N/S Hemisphere"));
  if((flds[6] != "W") && (flds[4] != "E"))
    return(reportBadMessage(msg, "Bad E/W Hemisphere"));
  
  string str_lat = flds[3];
  string str_lon = flds[5];
  string str_kts = flds[7];
  string str_hdg = flds[8];
  if(!isNumber(str_lat))
    return(reportBadMessage(msg, "Bad Lat"));
  if(!isNumber(str_lon))
    return(reportBadMessage(msg, "Bad Lon"));
  // New GPS modules fail to send speed and heading
  // sometimes when the GPS signal is poor.
  // Moving these checks below for now
  // extra warnings/ 

  double dbl_lat = latDDMMtoDD(str_lat);
  double dbl_lon = lonDDDMMtoDDD(str_lon);
  if(flds[4] == "S")
    dbl_lat = -dbl_lat;
  if(flds[6] == "W")
    dbl_lon = -dbl_lon;
  Notify("NAV_LAT", dbl_lat, "GPRMC");
  Notify("NAV_LON", dbl_lon, "GPRMC");
  Notify("NAV_LONG", dbl_lon, "GPRMC");
  
  double x, y;
  bool ok = m_geodesy.LatLong2LocalGrid(dbl_lat, dbl_lon, y, x);
  if(ok) {
    m_nav_x = x;
    m_nav_y = y;
    Notify("NAV_X", x, "GPRMC");
    Notify("NAV_Y", y, "GPRMC");
  }
  
  if(isNumber(str_kts)) {
    double dbl_kts = atof(str_kts.c_str());
    double dbl_mps = dbl_kts * 0.514444;
    m_nav_spd = dbl_mps;
    Notify("NAV_SPEED", dbl_mps, "GPRMC");
  }

  if(isNumber(str_hdg)) {
    double dbl_hdg = atof(str_hdg.c_str());
    m_nav_hdg = dbl_hdg;
    Notify("NAV_HEADING", dbl_hdg, "GPRMC");
  }
  return(true);
}





//---------------------------------------------------------
// Procedure: reportBadMessage()
  
bool VehX::reportBadMessage(string msg, string reason)
{
  reportRunWarning("Bad NMEA Msg: " + reason + ": " + msg);
  Notify("IVEHX_BAD_NMEA", reason + ": " + msg);
  return(false);
}

//---------------------------------------------------------
// Procedure: reportWarningsEvents()
//      Note: Get the AppCast-consistent events, warnings and
//            retractions from the sock ninja for posting

void VehX::reportWarningsEvents()
{
  // Part 1: Handle Event Messages()
  list<string> events = m_ninja.getEvents();
  list<string>::iterator p;  
  for(p=events.begin(); p!=events.end(); p++) {
    string event_str = *p;
    reportEvent(event_str);
  }

  // Part 2: Handle Warning Messages()
  list<string> warnings = m_ninja.getWarnings();
  list<string> thrust_warnings = m_thrust.getWarnings();
  warnings.splice(warnings.end(), thrust_warnings);
  for(p=warnings.begin(); p!=warnings.end(); p++) {
    string warning_str = *p;
    reportRunWarning(warning_str);
  }

  // Part 3: Handle Retraction Messages()
  list<string> retractions = m_ninja.getRetractions();
  for(p=retractions.begin(); p!=retractions.end(); p++) {
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


 
bool VehX::buildReport() 
{
  string str_max_rud  = doubleToStringX(m_max_rudder,1);
  string str_max_thr  = doubleToStringX(m_max_thrust,1);
  string str_max_both = str_max_rud + "/" + str_max_thr;
  string str_des_rud  = doubleToStringX(m_thrust.getRudder(),1);
  string str_des_thr  = doubleToStringX(m_thrust.getThrust(),1);
  string str_des_thrL = doubleToStringX(m_thrust.getThrustLeft(),1);
  string str_des_thrR = doubleToStringX(m_thrust.getThrustRight(),1);

  string str_sta_thr  = doubleToStringX(m_stale_threshold,1);
  string str_sta_ena  = boolToString(m_stale_check_enabled);

  string str_nav_x   = doubleToStringX(m_nav_x,1);
  string str_nav_y   = doubleToStringX(m_nav_y,1);
  string str_nav_hdg = doubleToStringX(m_nav_hdg,1);
  string str_nav_spd = doubleToStringX(m_nav_spd,1);


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
 
  list<string> summary_lines = m_ninja.getSummary();
  list<string>::iterator p;
  for(p=summary_lines.begin(); p!=summary_lines.end(); p++) {
    string line = *p;
    m_msgs << line << endl;
  }

  return(true);
}



