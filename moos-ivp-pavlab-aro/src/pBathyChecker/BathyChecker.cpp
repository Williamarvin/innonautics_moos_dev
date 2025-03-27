/************************************************************/
/*    NAME:                                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BathyChecker.cpp                                */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "BathyChecker.h"

using namespace std;

//---------------------------------------------------------
// Constructor

BathyChecker::BathyChecker()
{
}

//---------------------------------------------------------
// Destructor

BathyChecker::~BathyChecker()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool BathyChecker::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();

    if (key == "COMPLETED") {
      m_completed_names[sval] = 1;

      Notify("COMPLETED_RECEIVED", to_string(m_completed_names.size()));
    
    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool BathyChecker::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool BathyChecker::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool BathyChecker::OnStartUp()
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
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void BathyChecker::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("COMPLETED", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool BathyChecker::buildReport() 
{
  
  m_msgs << "Completed vehicles:" << endl;
  
  for (map<string, int>::iterator it = m_completed_names.begin(); it != m_completed_names.end(); ++it) {
    m_msgs << (*it).first << endl;
  }

  return(true);
}




