/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ThrusterAlloc.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ThrusterAlloc.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ThrusterAlloc::ThrusterAlloc()
{

  m_rud_time = MOOSTime();
  m_thr_time = MOOSTime();
  m_stale_thresh = 2.0; 
}

//---------------------------------------------------------
// Destructor

ThrusterAlloc::~ThrusterAlloc()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ThrusterAlloc::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if(key == "DESIRED_THRUST") {
      if (m_thruster.setThrust(msg.GetDouble()))
	m_thr_time = msg.GetTime(); 
      else
	reportRunWarning("Unhandled Mail: " + key);
    } else if (key == "DESIRED_RUDDER") {
      if (m_thruster.setRudder(msg.GetDouble()))
	m_rud_time = msg.GetTime();
      else
	reportRunWarning("Unhandled Mail: " + key);

    }else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ThrusterAlloc::OnConnectToServer()
{
  //registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ThrusterAlloc::Iterate()
{
  AppCastingMOOSApp::Iterate();

  double dt_rud = MOOSTime() - m_rud_time;
  double dt_thr = MOOSTime() - m_thr_time;
  bool rud_ok = (dt_rud < m_stale_thresh);
  bool thr_ok = (dt_thr < m_stale_thresh);

  if (rud_ok && thr_ok){
    bool calc_ok = m_thruster.calcDiffThrust();

    if (calc_ok){
      Notify("DESIRED_THRUST_L", m_thruster.getThrustLeft());
      Notify("DESIRED_THRUST_R", m_thruster.getThrustRight()); 
    }
  }
  

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ThrusterAlloc::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  bool got_drive_mode = false;
  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "foo") {
      handled = true;
    }
    else if(param == "mode") {
      handled = m_thruster.setDriveMode(value);
      if (handled)
	got_drive_mode = true; 
    }
    else if(param == "rev_factor") {
      handled = m_thruster.setRevFactor(value); 
    }
    else if(param == "max_rudder") {
      handled = m_thruster.setMaxRudder(value); 
    }
    else if(param == "max_thrust") {
      handled = m_thruster.setMaxThrust(value); 
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  if (!got_drive_mode)
    m_thruster.setDriveMode("aggro");

  Notify("THRUST_MODE_DIFFERENTIAL", "TRUE");
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ThrusterAlloc::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("DESIRED_THRUST", 0);
  Register("DESIRED_RUDDER", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool ThrusterAlloc::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  /*ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();
  */
  return(true);
}




