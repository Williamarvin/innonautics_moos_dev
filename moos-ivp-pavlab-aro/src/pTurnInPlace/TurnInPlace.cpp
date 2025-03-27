/************************************************************/
/*    NAME: Filip Stromstad                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: TurnInPlace.cpp                                 */
/*    DATE: October 15th, 2024                              */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "TurnInPlace.h"

#include "XYFormatUtilsPoint.h"
#include "AngleUtils.h"


using namespace std;

//---------------------------------------------------------
// Constructor()

TurnInPlace::TurnInPlace()
{
  m_desired_point = XYPoint(0,0);
  m_desired_heading = -1;
  m_zero = 0;

  m_os_name = "";
  m_mode = "sim";

  m_update_variable = "TIP_UPDATE";
  m_condition_variable = "TURN_IN_PLACE";
  m_condition_value = "true";
  m_condition_satisfied = false;
  m_completed = false;
  m_endflag_variable = "TURN_IN_PLACE";
  m_endflag_value = "false";


  m_osx = 0;
  m_osy = 0;
  m_heading = 0;
  m_heading_variable = "NAV_HEADING";
  m_last_heading_diff = 0;
  m_cumulative_heading_diff = 0;
  P_effort = 0;
  I_effort = 0;
  D_effort = 0;
}

//---------------------------------------------------------
// Destructor

TurnInPlace::~TurnInPlace()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool TurnInPlace::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == m_condition_variable) {
        if (sval == m_condition_value){
          m_condition_satisfied = true;
          m_completed = false;
        } else{
          m_condition_satisfied = false;
        }
      } else if(key == "NAV_X") {
        m_osx = dval;
      } else if(key == "NAV_Y") {
        m_osy = dval;
      } else if(key == m_heading_variable) {
        m_heading = dval;
      } else if(key == m_update_variable){
        string sval_copy = sval;
        string update_variable = biteStringX(sval_copy, '=');
        string update_value = sval_copy;

        if (update_variable == "heading"){
          m_desired_heading = stod(update_value);
          m_desired_point = XYPoint(-1,-1);
        } else if (update_variable == "point"){
          m_desired_point = string2Point(update_value);
          m_desired_heading = -1;
        } else {
          reportRunWarning("Unhandled update: " + sval);
        }
      }

     else if(key != "APPCAST_REQ") 
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool TurnInPlace::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool TurnInPlace::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // TODO: Add multiple conditions and endflags!
  if (!m_condition_satisfied || m_completed){
    Notify("ROTATE_SPEED", m_zero);
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  double desired_heading = m_desired_heading;
  if (desired_heading == -1){
    desired_heading = relAng(m_osx, m_osy, m_desired_point.x(), m_desired_point.y());
  }

  double heading_diff = desired_heading - m_heading;
  
  if (heading_diff > 180){
    heading_diff = heading_diff - 360;
  } else if (heading_diff < -180){
    heading_diff = heading_diff + 360;
  }


  if (m_mode == "sim"){
    double P = 0.1;
    // double P = 1;
    // double I = 0.1;
    double D = 0;

    double effort = P * heading_diff + D * (heading_diff - m_last_heading_diff);

    double eps = 1*P;
    if (abs(effort) > eps){
      Notify("ROTATE_SPEED", effort);
    } else {
      Notify("ROTATE_SPEED", m_zero);
      Notify(m_endflag_variable, m_endflag_value);
      m_completed = true;
    }
  } else if (m_mode == "m300"){
    double P = 1;
    double D = 0;
    double I = 0.003;

    double max_integral = 25;

    // Reset integral term if the heading difference is small or if the sign changes
    if (abs(heading_diff) < 5){
      m_cumulative_heading_diff = 0;
    } else if (heading_diff * m_last_heading_diff < 0){
      m_cumulative_heading_diff = 0;
    } else if (abs(I*m_cumulative_heading_diff) < max_integral){
      m_cumulative_heading_diff += heading_diff;
    }

    P_effort = P * heading_diff;
    I_effort = I * m_cumulative_heading_diff;
    D_effort = D * (heading_diff - m_last_heading_diff);

    double effort = P_effort + I_effort + D_effort;
    double eps = 1*P;


    // TODO:
    // 1. Calculate a right and left thrust
    // 2. Send directly to the vehicle
    // 3. Adapt thrust coefficients based on movement of vehicle
    double thrust_max_reverse = 100;
    double thust_max_forward = 30;
    double thrust_left = 0;
    double thrust_right = 0;

    double effort_normalized = abs(effort) / 180;

    if (abs(effort) < eps){
      // do nothing
    } else {
      if (effort > 0){
        // Turn right
        thrust_left = thust_max_forward*effort_normalized;
        thrust_right = -thrust_max_reverse*effort_normalized;
      } else {
        // Turn left
        thrust_left = -thrust_max_reverse*effort_normalized;
        thrust_right = thust_max_forward*effort_normalized;
      }
    }

    Notify("REQ_THRUSTER_L", thrust_left);
    Notify("REQ_THRUSTER_R", thrust_right);
  }



  m_last_heading_diff = heading_diff;

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool TurnInPlace::OnStartUp()
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
    if(param == "vname") {
      m_os_name = value;
      handled = true;
    } else if (param == "mode"){
      m_mode = tolower(value);
      
      if (m_mode == "sim"){
        m_heading_variable = "NAV_HEADING";
        handled = true;
      } else if (m_mode == "m300"){
        m_heading_variable = "COMPASS_HEADING_RAW";
        handled = true;
      }

    } else if (param == "heading"){
      m_desired_heading = stod(value);
      m_desired_point = XYPoint(-1,-1);
      handled = true;
    } else if (param == "point"){
      m_desired_point = string2Point(value);
      m_desired_heading = -1;
      handled = true;
    } else if (param == "condition"){
      // Variable and value separated by "=="
      string condition_variable = value;
      string condition_value = value;
      m_condition_variable = biteStringX(condition_variable, '=');
      m_condition_value = rbiteStringX(condition_value, '=');
      handled = true;
    } else if (param == "endflag"){
      // Variable and value separated by "=="
      string endflag_variable = value;
      string endflag_value = value;
      m_endflag_variable = biteStringX(endflag_variable, '=');
      m_endflag_value = rbiteStringX(endflag_value, '=');
      handled = true;
    } else if (param == "updates"){
      m_update_variable = value;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  Register(m_heading_variable, 0);
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void TurnInPlace::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  // Register("NAV_HEADING", 0);
  // Register("COMPASS_HEADING_RAW", 0);
  // Register(m_heading_variable, 0);
  Register(m_update_variable, 0);
  Register(m_condition_variable, 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool TurnInPlace::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: TurnInPlace.cpp                       " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "State variables" << endl;
  m_msgs << "  m_os_name: " << m_os_name << endl;
  m_msgs << "  m_desired_heading: " << m_desired_heading << endl;
  m_msgs << "  m_osx: " << m_osx << endl;
  m_msgs << "  m_osy: " << m_osy << endl;
  m_msgs << "  m_heading: " << m_heading << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "PID variables" << endl;
  m_msgs << "  P_effort: " << P_effort << endl;
  m_msgs << "  I_effort: " << I_effort << endl;
  m_msgs << "  D_effort: " << D_effort << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Configuration variables" << endl;
  m_msgs << "  m_mode: " << m_mode << endl;
  m_msgs << "  m_desired_point: " << m_desired_point.get_spec() << endl;
  m_msgs << "  m_desired_heading: " << m_desired_heading << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Behaviour-like variables" << endl;
  m_msgs << "  Heading variable: " << m_heading_variable << endl;
  m_msgs << "  Update variable: " << m_update_variable << endl;
  m_msgs << "  Condition: " << m_condition_variable << " == " << m_condition_value << endl;
  m_msgs << "  Condition satisfied: " << m_condition_satisfied << endl;
  m_msgs << "  Completed: " << m_completed << endl;
  m_msgs << "  Endflag: " << m_endflag_variable << " == " << m_endflag_value << endl;
  m_msgs << "============================================" << endl;

  return(true);
}




