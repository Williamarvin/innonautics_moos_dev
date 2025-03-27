/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: HeronSim.cpp                                         */
/*    DATE: Mar 17th 2020                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <sys/time.h>
#include "MBUtils.h"
#include "HeronSim.h"
#include "LatLonFormatUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor

HeronSim::HeronSim()
{
  // Init state variables  
  m_LastValidPYDIRtime = 0;

  m_cur_thrust_lft  = 0;
  m_cur_thrust_rgt  = 0;

  m_curr_spd        = 0;
  m_curr_hdg        = 0;
  m_curr_lat        = 0;
  m_curr_lon        = 0;
  m_curr_hdg_rate   = 0; 

  m_CPNVGlastTime   = 0;
  m_CPRBSlastTime   = 0;
  m_GPRMClastTime   = 0;
  m_CPNVRlastTime   = 0;
  m_CPRCMlastTime   = 0; 

  m_last_hdg        = 0;
  m_last_hdg_time   = 0;
  m_mag_declination_deg = -14.2; 
}


//---------------------------------------------------------
// Procedure: OnNewMail()

bool HeronSim::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);
  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key  = msg.GetKey();
    double dval = msg.GetDouble();
    double mtime  = msg.GetTime();
    
    if(key == "NAV_LAT")
      m_curr_lat = dval;
    else if(key == "NAV_LONG")
      m_curr_lon = dval;
    else if(key == "NAV_HEADING") {
      m_last_hdg = m_curr_hdg;  // will be zero on startup
      m_curr_hdg = dval;

      updateHeadingRate(mtime);
     
    } else if(key == "NAV_SPEED")
      m_curr_spd = dval;
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool HeronSim::Iterate()
{
  AppCastingMOOSApp::Iterate();
  m_curr_time = MOOSTime();

  if(m_ninja.getState() != "connected")
    m_ninja.setupConnection();

  if(m_ninja.getState() == "connected") {

    // Incoming
    ReadIncoming();

    // Outgoing
    ParseCurrentTime();

#if 1
    // comment out to test client nonblocking with nothing to read
    if(m_curr_time - m_CPNVGlastTime > 0.2) // 5Hz
      PublishCPNVG();
    if(m_curr_time - m_CPRBSlastTime > 2)   // 0.5Hz
      PublishCPRBS();
    if(m_curr_time - m_GPRMClastTime > 0.2) // 5Hz
      PublishGPRMC();
    if(m_curr_time - m_CPNVRlastTime > 0.1) // 10Hz
      PublishCPNVR();
    if(m_curr_time - m_CPRCMlastTime > 0.1) // 10Hz
      PublishCPRCM(); 
    
#endif
  }

  checkForStalePYDIR();
  reportWarningsEvents();
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool HeronSim::OnConnectToServer()
{
  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool HeronSim::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    
    bool handled = false;
    if((param == "port") && isNumber(value)) {
      int port = atoi(value.c_str());
      handled = m_ninja.setPortNumber(port);
    }
    else if(param == "comms_type")
      handled = m_ninja.setCommsType(value);
    else if(param == "ip_addr")
      handled = m_ninja.setIPAddr(value);
    else if(param == "mag_declination_deg")
      handled = setDoubleOnString(m_mag_declination_deg, value); 
 
    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  Notify("THRUST_MODE_DIFFERENTIAL", "true");
  RegisterVariables();
  return(true);
}


//---------------------------------------------------------
// Procedure: RegisterVariables()

void HeronSim::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  
  Register("NAV_LAT");
  Register("NAV_LONG");
  Register("NAV_HEADING");
  Register("NAV_SPEED");
}

//---------------------------------------------------------
// Procedure: publishThrustLR()

void HeronSim::publishThrustLR(double left, double right)
{
  Notify("DESIRED_THRUST_L", left);
  Notify("DESIRED_THRUST_R", right);
}

//---------------------------------------------------------
// Procedure: ReadIncoming()

bool HeronSim::ReadIncoming()
{
  list<string> nmea_msgs = m_ninja.getSockMessages();

  list<string>::iterator p;  
  for(p=nmea_msgs.begin(); p!=nmea_msgs.end(); p++) {
    string nmea_msg = *p;
    DealWithNMEA(nmea_msg);
  }

  return(true);
}


//---------------------------------------------------------
// Procedure: reportWarningsEvents()

void HeronSim::reportWarningsEvents()
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


//---------------------------------------------------------
// Procedure: DealWithNMEA()

bool HeronSim::DealWithNMEA(string nmea)
{
  string nmea_orig = nmea;
  string key = biteStringX(nmea, ',');

  if(key == "$PYDIR")
    return(ReceivePYDIR(nmea_orig));
  
  reportRunWarning("Received input from socket that cannot be processed: "+ nmea);
  reportRunWarning("Bad key: [" + key + "]");
  return(true);
}

//---------------------------------------------------------
// Procedure: PublishToClient()

void HeronSim::PublishToClient(string str)
{
  if(str.empty())
    return;

  m_ninja.sendSockMessage(str);
}

//---------------------------------------------------------
// Procedure: ParseCurrentTime()

bool HeronSim::ParseCurrentTime()
{
  timeval tp;
  gettimeofday(&tp, NULL);
  time_t now               = tp.tv_sec;
  tm *t                    = localtime(&now);
  unsigned short hour      = t->tm_hour;
  unsigned short minute    = t->tm_min;
  float second             = (float) t->tm_sec + tp.tv_usec / 1000000.0;
  
  m_utc_now.setTime((unsigned int)hour, (unsigned int)minute, second);
  
  return(true);
}

//---------------------------------------------------------
// Procedure: PublishCPRBS()
//      Note: Battery Voltage
//      Note: $CPRBS,time,id_battery,v_batt_stack,v_batt_min,
//            v_batt_max,temp_celsius*HH

bool HeronSim::PublishCPRBS()
{
  string utc = m_utc_now.getTimeStr();
  //   battery_id = 1
  // v_batt_stack = 15.2
  //   v_batt_min = 15.1
  //   v_batt_max = 15.3
  // temp_celcius = 19.8
  string msg = "CPRBS," + utc + ",1,15.2,15.1,15.3,19.8";
  msg = "$" + msg + "*" + checksumHexStr(msg) + "\r\n";

  PublishToClient(msg);
  m_CPRBSlastTime = MOOSTime(); 
  return(true);
}

//---------------------------------------------------------
// Procedure: PublishCPNVG()
//      Note: $CPNVG,time,lat,N,lon,W,qual,alt,dep,hdg,roll,pitch,navtime*HH

bool HeronSim::PublishCPNVG()
{
  string utc = m_utc_now.getTimeStr();
  string lat = latDDtoDDMM(m_curr_lat);
  string lon = lonDDDtoDDDMM(m_curr_lon);
  string msg = "CPNVG," + utc + "," + lat + ",N," + lon + ",W,1,,,";
  msg += doubleToStringX(m_curr_hdg, 2) + ",,," + utc;
  msg = "$" + msg + "*" + checksumHexStr(msg) + "\r\n";

  PublishToClient(msg);
  m_CPNVGlastTime = MOOSTime(); 
  return(true);
}


//---------------------------------------------------------
// Procedure: PublishGPRMC()
//      Note: $GPRMC,time,A,lat,N,lon,W,kts,crs,date,magvar,E*HH

bool HeronSim::PublishGPRMC()
{
  string kts = doubleToStringX(1.94384 * m_curr_spd);
  string hdg = doubleToStringX(m_curr_hdg);
  string utc = m_utc_now.getTimeStr();
  string lat = latDDtoDDMM(m_curr_lat);
  string lon = lonDDDtoDDDMM(m_curr_lon);
  string msg = "GPRMC," + utc + ",A," + lat + ",N," + lon + ",W," + kts;
  msg += "," + hdg + ",291263,0,0,E";
  msg = "$" + msg + "*" + checksumHexStr(msg) + "\r\n";
  
  PublishToClient(msg);
  m_GPRMClastTime = MOOSTime(); 
  return(true);
}

//---------------------------------------------------------
// Procedure: PublishCPNVR()
// note: $CPNVR,time,vel_east,vel_north,vel_down,rate_pitch,rate_roll,rate_yaw*CS

bool HeronSim::PublishCPNVR()
{

  double deg_to_rad = M_PI/180.0; 
  double vel_east  = m_curr_spd * sin(m_curr_hdg * deg_to_rad);
  double vel_north = m_curr_spd * cos(m_curr_hdg * deg_to_rad);
  
  string utc = m_utc_now.getTimeStr();
  string str_vel_east  = doubleToStringX(vel_east);
  string str_vel_north = doubleToStringX(vel_north); 
  string vel_down   = "0";
  string rate_pitch = "0";
  string rate_roll  = "0";
  string str_rate_yaw   = doubleToStringX(m_curr_hdg_rate);
  
  string msg = "CPNVR," + utc + "," + str_vel_east + "," + str_vel_north + "," + vel_down;
  msg += "," + rate_pitch + "," + rate_roll + "," + str_rate_yaw;
  msg = "$" + msg + "*" + checksumHexStr(msg) + "\r\n";

  PublishToClient(msg);
  m_CPNVRlastTime = MOOSTime(); 
  return(true);
}

//---------------------------------------------------------
// Procedure: PublishCPRCM()
//      Note:  $CPRCM,time,id,hdg,pitch,roll,nav_time*CS
bool HeronSim::PublishCPRCM()
{
  string utc = m_utc_now.getTimeStr();
  // Subtract the delination for sensor simulation. 
  string str_hdg = doubleToStringX(m_curr_hdg - m_mag_declination_deg);
  
  string msg = "CPRCM," + utc + ",1," + str_hdg + ",0,0,,";
  msg = "$" + msg + "*" + checksumHexStr(msg) + "\r\n";

  PublishToClient(msg);
  m_CPRCMlastTime = MOOSTime(); 
  return(true);
}


//---------------------------------------------------------
// Procedure: ReceivePYDIR()
//   
//   $PYDIR,<1>,<2>*hh<CR><LF>
//   <1>  [DesThrustPctL] Desired pct thrust for portside motor, -100 to 100.
//   <2>  [DesThrustPctR] Desired pct thrust for starboard motor, -100 to 100.

bool HeronSim::ReceivePYDIR(string nmea)
{
  if(!isValidNMEA(nmea))
    reportRunWarning("Error in incoming PYDIR sentence:[" + nmea + "]");

  if(strBegins(nmea, "$PYDIR,"))
    biteStringX(nmea, ',');

  rbiteString(nmea, '*');
  vector<string> svector = parseString(nmea, ',');
  if(svector.size() != 2)
    return(false);

  m_LastValidPYDIRtime = m_curr_time;

  m_cur_thrust_lft = atof(svector[0].c_str());
  m_cur_thrust_rgt = atof(svector[1].c_str());
  publishThrustLR(m_cur_thrust_lft, m_cur_thrust_rgt);
  return(true);
}

//---------------------------------------------------------
// Procedure: checkForStalePYDIR()

void HeronSim::checkForStalePYDIR()
{
  if(m_LastValidPYDIRtime == 0)
    return;

  if((m_curr_time - m_LastValidPYDIRtime) > WATCHDOG_TIME) {
    reportRunWarning("Watchdog timeout, halting motors.");
    m_LastValidPYDIRtime = 0;
    m_cur_thrust_lft = 0;
    m_cur_thrust_rgt = 0;
    publishThrustLR(m_cur_thrust_lft, m_cur_thrust_rgt);
  }
  retractRunWarning("Watchdog timeout, halting motors.");
}

//---------------------------------------------------------
// Procedure: isValidNMEA()

bool HeronSim::isValidNMEA(string str)
{
  if((str.length()==0) || (str[0]!='$') || !strContains(str, '*'))
    return(false);

  str = str.substr(1); // chop off the leading dollar sign

  string tail = rbiteString(str, '*');
  if(tail.length() < 2)
    return(false);
  string hexstr1 = tail.substr(0,2);
  string hexstr2 = checksumHexStr(str);

  if(hexstr1 != hexstr2)
    return(false);
    
  return(true);
}


void HeronSim::updateHeadingRate(double time) {

  double dt = time - m_last_hdg_time; // will be very large on startup
  if (abs(dt) < 0.001){
      dt = 0.1; 
      Notify("DEBUG_FIXED_TIME", "here"); 
  }
  double rad_to_deg = 180.0/M_PI;
  double deg_to_rad = M_PI/180.0;
  
  // Unit vectors
  double a1 = sin(deg_to_rad*m_curr_hdg); 
  double a2= cos(deg_to_rad*m_curr_hdg);  
  
  double b1 = sin(deg_to_rad*m_last_hdg);     
  double b2 = cos(deg_to_rad*m_last_hdg);     

  double k = a1*b2 - a2*b1;  // a cross b
  double dot_a_b = a1*b1+ a2*b2;

  double delta_theta;
  if (k >=0){
    delta_theta = rad_to_deg * acos(dot_a_b) * (1.0);
  } else {
    delta_theta = rad_to_deg * acos(dot_a_b) * (-1.0);
  }

  if (isnan(delta_theta)){
    Notify("DEBUG_FIXED_DELTA_THETA", "here"); 
    delta_theta = 0.0; 
  }
  m_curr_hdg_rate = delta_theta / dt;
  m_last_hdg_time = time;
  
}

//---------------------------------------------------------
// Procedure: buildReport()
//
//    Currently connected to a backseat client on port 29500.
//    NMEA Sentences:
//    Number Tx: 8675      Sec since last Tx:
//    Number Rx: 309       Sec since last Rx:
//    Client command uses: PYDIR
//    Rx and Tx NMEA sentences are being shown in the console window.

bool HeronSim::buildReport()
{
  //=================================================
  // Part 1: Handle the case of being disconnected
  //=================================================
  //m_msgs << "Status:                          " << endl;
  //m_msgs << "  Type:   " << m_ninja.getType()   << endl;
  //m_msgs << "  State:  " << m_ninja.getState()  << endl;
  //m_msgs << "  IPAddr: " << m_ninja.getIPAddr() << "(client)" << endl;
  //m_msgs << "  Port:   " << intToString(m_port) << endl;
  //m_msgs << "                                 " << endl;

  //m_msgs << "---------------------------------" << endl;
  //m_msgs << "NMEA sentences:" << endl;
  list<string> summary_lines = m_ninja.getSummary();
  list<string>::iterator p;
  for(p=summary_lines.begin(); p!=summary_lines.end(); p++) {
    string line = *p;
    m_msgs << line << endl;
  }

  list<string> error_lines = m_ninja.getSummaryStatErrors();
  for(p=error_lines.begin(); p!=error_lines.end(); p++) {
    string line = *p;
    m_msgs << line << endl;
  }

 
  m_msgs << endl;
  m_msgs << "Latest Thrust Values:     " << endl;
  m_msgs << "  THRUST_L: " << m_cur_thrust_lft << endl;
  m_msgs << "  THRUST_R: " << m_cur_thrust_rgt << endl;

  return(true);
}


