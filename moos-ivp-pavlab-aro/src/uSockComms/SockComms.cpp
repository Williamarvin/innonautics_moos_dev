/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: SockComms.cpp                                        */
/*    DATE: March 22nd 2020                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "SockComms.h"

using namespace std;

//---------------------------------------------------------
// Constructor

SockComms::SockComms()
{
  m_ninja = SockNinja("client", 29500);
  m_message = "45,55";
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SockComms::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval   = msg.GetString(); 

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if(key == "POST_SOCK_MSG") 
      handleMailPostSockMsg(sval);
    
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SockComms::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SockComms::Iterate()
{
  AppCastingMOOSApp::Iterate();
  if(m_ninja.getState() != "connected")
    m_ninja.setupConnection();
  
  if(m_ninja.getState() == "connected") {
    sendMessagesToSocket();
    readMessagesFromSocket();
  }

  reportWarningsEvents();
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SockComms::OnStartUp()
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
    else if(param == "ip_addr")
      handled = m_ninja.setIPAddr(value);
    else if(param == "message")
      handled = setNonWhiteVarOnString(m_message, value);
    else if(param == "comms_type")
      handled = m_ninja.setCommsType(value);

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: handleMailPostSockMsg()

void SockComms::handleMailPostSockMsg(string msg)
{
  if(msg == "")
    return;
  m_sock_msgs.push_front(msg);
}


//---------------------------------------------------------
// Procedure: registerVariables

void SockComms::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("POST_SOCK_MSG", 0);
}



//---------------------------------------------------------
// Procedure: sendMessagesToSocket()

void SockComms::sendMessagesToSocket()
{
  string msg = "PYDIR,";
  msg += m_message;
  msg = "$" + msg + "*" + checksumHexStr(msg) + "\r\n";

  m_latest_tx_msg = msg;
  m_ninja.sendSockMessage(msg);
}

//---------------------------------------------------------
// Procedure: readMessagesFromSocket()

void SockComms::readMessagesFromSocket()
{
  list<string> incoming_msgs = m_ninja.getSockMessages();
  list<string>::iterator p;
  for(p=incoming_msgs.begin(); p!=incoming_msgs.end(); p++) {
    m_latest_rx_msg = *p;
    Notify("READ_SOCK_MESSAGE", m_latest_rx_msg);
  }

}

//---------------------------------------------------------
// Procedure: reportWarningsEvents()
//      Note: Get the AppCast-consistent events, warnings and
//            retractions from the sock ninja for posting

void SockComms::reportWarningsEvents()
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


//------------------------------------------------------------
// Procedure: buildReport()

bool SockComms::buildReport() 
{
  list<string> summary_lines = m_ninja.getSummary();
  list<string>::iterator p;
  for(p=summary_lines.begin(); p!=summary_lines.end(); p++) {
    string line = *p;
    m_msgs << line << endl;
  }

  return(true);
}



