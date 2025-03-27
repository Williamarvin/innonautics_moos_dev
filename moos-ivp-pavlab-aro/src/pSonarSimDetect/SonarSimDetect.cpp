/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SonarSimDetect.cpp                                        */
/*    DATE: May 26, 2020                                */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SonarSimDetect.h"
#include "LogicUtils.h"
#include "NodeRecordUtils.h"
#include <cmath>
using namespace std;

//---------------------------------------------------------
// Constructor

SonarSimDetect::SonarSimDetect()
{
  m_sonar_rad=10;
  m_target_x=0;
  m_target_y=0;
  m_osx=0;
  m_osy=0;
  m_os_speed=0;
  m_target_time=-10;
  m_target_depth=10;
  m_last_report_time=0;
  m_time_limit=false;
  m_time_l=2;
  m_sigma=0;
  m_beam_angle=30;
  m_gotx=false;
  m_goty=false;
  m_hz=1;
  m_active=true;
}

//---------------------------------------------------------
// Destructor

SonarSimDetect::~SonarSimDetect()
{
}
//---------------------------------------------------------
// Procedure: Set target Name

bool SonarSimDetect::setTarget(string name)
{
  m_target_name=tolower(name);
  Notify("FIRST",m_target_name);
  return true;
}

//---------------------------------------------------------
// Procedure: handleNodeReport()
bool SonarSimDetect::handleNodeReport(string node_report)
{

  if(!m_active)
    return(true);
  
  NodeRecord record = string2NodeRecord(node_report);
  bool got_detection=false;
  if(!record.valid())
    return(false);
  
  string vname = record.getName(); 
  double speed= record.getSpeed();
  double heading= record.getHeading();
  if(tolower(vname)==tolower(m_target_name)){ 
    double time=record.getTimeStamp();
    double depth=record.getDepth();
    if(time-m_target_time>=1/m_hz){
      m_target_x=record.getX();
      m_target_y=record.getY();
      m_target_time=time;
      if(depth>1){
        m_target_depth=depth;
      }
      setSonarRad();
      got_detection=checkDetect();
    }
    Notify("SSD_GOT_CONTACT_REPORT", "true");
  }
  //If the Sonar has a detection then Send Detection Report
  if(got_detection){
    string d_rep=doubleToStringX(m_osx);
    d_rep=d_rep+";"+doubleToStringX(m_osy);
    d_rep=d_rep+";"+doubleToStringX(m_target_time);
    d_rep=d_rep+";"+doubleToStringX(heading);
    d_rep=d_rep+";"+doubleToStringX(speed);
    if(m_time_limit){
      m_time_l= (2*(1+m_sonar_rad))/(speed-m_os_speed);
      if(m_target_time-m_last_report_time>m_time_l){
        Notify("DETECTION_REPORT", d_rep);
        m_last_report_time=m_target_time;
	sendPulse();
	postDetectFlags();
      }
    }
    else{
      Notify("DETECTION_REPORT", d_rep);
      sendPulse();
      postDetectFlags();
    }

  }
  return true;
}

//---------------------------------------------------------
// Procedure: CheckDetect()
//Use Guassian distribution for sonar detection
bool SonarSimDetect::checkDetect()
{
  bool hit=false;
  //Verify own ships position is updated from initialization
  if(m_gotx&&m_goty){
    double dist = hypot(m_osx-m_target_x, m_osy-m_target_y);
    Notify("SSD_TARGET_RANGE",dist); 
    if(dist<m_sonar_rad+1){
      double exponent=(dist*dist)/(2*(m_sigma*m_sigma));
      double p=exp(-1*exponent);
      double result=(float) rand()/RAND_MAX;
      if (result<p){
          hit=true;
      }
    }
  }
  return hit;
}

//---------------------------------------------------------
// Procedure: setSonarRad()
//Establish the sonar detection threshold from sonar geometry
void SonarSimDetect::setSonarRad()
{
  m_sonar_rad=m_target_depth*tan((m_beam_angle/2)*M_PI/180);
  if(m_sonar_rad>2){
  m_sigma=m_sonar_rad/3;
  }
  else{
    m_sigma=m_sonar_rad;
  }
  Notify("SSD_SONAR_RAD",m_sonar_rad);
  return;
}

//---------------------------------------------------------
// Procedure: postDetectFlags()
void SonarSimDetect::postDetectFlags()
{
  std::map<std::string, double>::iterator it;
  for (it = m_detect_double_flags.begin(); it != m_detect_double_flags.end(); it++){
    Notify(it->first, it->second); 
  }
  return; 
}


//---------------------------------------------------------
// Procedure: handleDetectFlagDouble()
bool SonarSimDetect::handleDetectFlagDouble(std::string line)
{
  string var = toupper(biteStringX(line, '='));
  string val = line;
  double val_dbl = 0.0;

  if (var == "")
    return(false);

  bool ok = setDoubleOnString(val_dbl, val);
  if (!ok)
    return(false); 

  m_detect_double_flags[var] = val_dbl;
  return(true);
}


//--------------------------------------------------------
// Procedure: sendPulse()
void SonarSimDetect::sendPulse() {
  XYRangePulse pulse;
  pulse.set_x(m_osx);                
  pulse.set_y(m_osy);                
  pulse.set_label("intruder_detect_pulse");
  pulse.set_rad(m_sigma);
  pulse.set_time(MOOSTime());       
  pulse.set_color("edge", "yellow");
  pulse.set_color("fill", "red");
  pulse.set_duration(60.0);

  string spec = pulse.get_spec();
  Notify("VIEW_RANGE_PULSE", spec);
  return; 
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SonarSimDetect::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
    if(key == "NODE_REPORT"){ 
      string sval  = msg.GetString();
      handleNodeReport(sval);
    }
    else if(key == "NAV_X"){ 
      double dval  = msg.GetDouble();
      m_osx=dval;
      m_gotx=true;
    }
    else if(key == "NAV_Y"){ 
      double dval  = msg.GetDouble();
      m_osy=dval;
      m_goty=true;
    }
    else if(key == "NAV_SPEED"){ 
      double dval  = msg.GetDouble();
      m_os_speed=dval;
    }
    else if(key == "ACTIVE_DETECT"){
      setBooleanOnString(m_active, msg.GetString());
    }
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SonarSimDetect::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SonarSimDetect::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SonarSimDetect::OnStartUp()
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
    if(param == "beam_angle"){ 
      handled = setNonNegDoubleOnString(m_beam_angle, value);
    }
    else if(param == "target_name") {
      handled = setTarget(value);
    }
    else if(param == "target_depth"){
      handled = setNonNegDoubleOnString(m_target_depth, value);
    }
    else if(param == "enforce_limit"){
      handled = setBooleanOnString(m_time_limit, value);
    }
    else if(param == "sonar_frequency"){
      handled = setNonNegDoubleOnString(m_hz, value);
    }
    else if(param == "detect_flag_double"){
      handled = handleDetectFlagDouble(value); 
    }
    
    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void SonarSimDetect::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
    Register("NODE_REPORT", 0);
    Register("NAV_X", 0);
    Register("NAV_Y", 0);
    Register("NAV_SPEED", 0);
    Register("ACTIVE_DETECT",0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool SonarSimDetect::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "Target Name = "<<m_target_name<<endl;
  m_msgs << "Target Depth = "<<m_target_depth<<endl;
  m_msgs << "Sonar Radius = "<<m_sonar_rad<<endl;
  m_msgs << "Time Limit Between Detections = "<< m_time_l <<endl;

  return(true);
}




