/************************************************************/
/*    NAME: Mikala Molina                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RequestPoints.cpp                                        */
/*    DATE: June 1 2023                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "RequestPoints.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

RequestPoints::RequestPoints()
{
  m_ready_to_receive = false;
  m_need_swimmer = false;
}

//---------------------------------------------------------
// Destructor

RequestPoints::~RequestPoints()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool RequestPoints::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    double dval  = msg.GetDouble();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "SWIMMERS_REMAINING"){
      m_swimmers_remaining = dval;
     } 
     else if(key == "SEND_POINTS"){
      m_ready_to_receive = true;
     }

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool RequestPoints::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool RequestPoints::Iterate()
{
  AppCastingMOOSApp::Iterate();
  if((m_swimmers_remaining < m_min_swimmers)&&(m_ready_to_receive)){
    m_need_swimmer = true;
    Notify("SWIMMER_REQUEST", m_vname);
    cout << "Swimmer Request Sent" << endl;
  }
  else if(m_swimmers_remaining >= m_min_swimmers){
    m_need_swimmer = false;
    cout << "no swimmers needed" << endl;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool RequestPoints::OnStartUp()
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
      m_vname = value;
      handled = true;
    }

    else if(param == "min_swimmers") {
      m_min_swimmers = stof(value);
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void RequestPoints::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("SWIMMERS_REMAINING", 0);
  Register("SEND_POINTS", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool RequestPoints::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: RequestPoints.cpp                    " << endl;
  m_msgs << "============================================" << endl;

m_msgs <<"Vehicle Name: " << m_vname << endl;
m_msgs <<"Ready to Receive: " << m_ready_to_receive << endl;
m_msgs <<"Swimmers Remaining: " << m_swimmers_remaining << endl;
m_msgs <<"Minimum Swimmers: " << m_min_swimmers << endl;
m_msgs <<"Need Swimmer?: " << m_need_swimmer << endl;

  return(true);
}




