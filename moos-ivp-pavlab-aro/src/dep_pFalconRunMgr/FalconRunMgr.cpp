/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: FalconRunMgr.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "FalconRunMgr.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

FalconRunMgr::FalconRunMgr()
{
  m_midpoint_flag_var_name = "MIDFLAG";
  m_loop_id_var_name       = "START_LOOP";
  
  m_loops_before_change_rad = 4;
  m_loops_before_induce_fault = 2;
  m_loops_before_return = 6;
  m_curr_loop_id = 0;

  m_sent_fault = false;
  m_sent_rad_change = false;
  m_sent_return = false;

  m_time_delay_rad_msg = 30.0; 

}

//---------------------------------------------------------
// Destructor

FalconRunMgr::~FalconRunMgr()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool FalconRunMgr::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if (key == m_loop_id_var_name) {
      // index starts at 0
      if (!handleLoopID(msg.GetString() )){
	  reportRunWarning("Loop ID not set correctly in mail");
      }
    } else if(key == m_midpoint_flag_var_name) {
       // Check if need to send out
      bool ok = true; 
      if ( (m_curr_loop_id+1) == m_loops_before_induce_fault){
	ok = sendFaultMsg(); 
	//} else if ( (m_curr_loop_id+1) == m_loops_before_change_rad) {
	//ok = sendRadChangeMsg();
      } else if ( (m_curr_loop_id+1) == m_loops_before_return) {
	Notify("LEG_RUNNING", "false"); 
	Notify("RETURN", "true");
	m_sent_return = true; 
	ok = true;
      }

      if (!ok) {
	reportRunWarning("Midpoint flag not handled, check msg output"); 
      }

    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool FalconRunMgr::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool FalconRunMgr::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  bool cond_1 = ( (m_curr_loop_id+1) == m_loops_before_change_rad);
  bool cond_2 = (MOOSTime() - m_time_rad_flag_rcv) > m_time_delay_rad_msg;

  bool ok; 
  if ( cond_1 && cond_2 && !m_sent_rad_change){
    ok = sendRadChangeMsg();
    
    if (!ok)
      reportRunWarning("Radius change flag not handled, check msg output");
 
  }

  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool FalconRunMgr::OnStartUp()
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
    if(param == "midpoint_flag_var_name") {
      handled = setNonWhiteVarOnString(m_midpoint_flag_var_name, toupper(value));
    }
    else if(param == "loop_id_var_name") {
      handled = setNonWhiteVarOnString(m_loop_id_var_name, toupper(value));
    }
    else if(param == "loops_before_inducing_fault") {
      double dbl_val;
      handled = setPosDoubleOnString(dbl_val, value);
      if (handled) {
	m_loops_before_induce_fault = static_cast<int>(dbl_val);
      }
    }
    else if(param == "loops_before_changing_rad") {
      double dbl_val;
      handled = setPosDoubleOnString(dbl_val, value);
      if (handled) {
	m_loops_before_change_rad = static_cast<int>(dbl_val);
      }
    }
    else if(param == "loops_before_return") {
      double dbl_val;
      handled = setPosDoubleOnString(dbl_val, value);
      if (handled) {
	m_loops_before_return = static_cast<int>(dbl_val);
      }
    }
    else if(param == "fault_msg") {
      m_fault_msg.insert(value);
      handled = true;
    }
    else if(param == "rad_change_msg") {
      m_rad_change_msg.insert(value);
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

void FalconRunMgr::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_midpoint_flag_var_name, 0);
  Register(m_loop_id_var_name, 0);

}


//------------------------------------------------------------
// Procedure: buildReport()

bool FalconRunMgr::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "Falcon Run Manager                          " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "             Loops started: " << intToString(m_curr_loop_id+1) << endl;
  m_msgs << "        Loops before fault: " << intToString(m_loops_before_induce_fault);
  if (m_sent_fault){
    m_msgs <<" : FAULT SENT" << endl;
  } else {
    m_msgs << endl; 
  }
  m_msgs << "Loops before radius change: " << intToString(m_loops_before_change_rad);
  if (m_sent_rad_change){
    m_msgs <<" : RAD CHANGE SENT" << endl;
  } else {
    m_msgs << endl; 
  }
  m_msgs << "       Loops before return: " << intToString(m_loops_before_return);
  if (m_sent_return){
    m_msgs <<" : RETURN SENT" << endl;
  } else {
    m_msgs << endl; 
  }
  
  std::set<std::string>::iterator it;
  m_msgs << "============================================" << endl;
  m_msgs << "      Unique Fault Msgs:                           " << endl;
  for (it = m_fault_msg.begin(); it != m_fault_msg.end(); ++it){
    m_msgs << "               * " << *it << endl; 
  }
  
  m_msgs << "                                            " << endl;
  m_msgs << " Unique Rad Change Msgs:                           " << endl;
    for (it = m_rad_change_msg.begin(); it != m_rad_change_msg.end(); ++it){
    m_msgs << "               * " << *it << endl; 
  }
  
  return(true);
}


  
bool FalconRunMgr::sendFaultMsg()
{
  std::set<std::string>::iterator it;
  bool ok = false;
  double dbl_val;
  std::string msg;
  std::string var_name;
  
  for (it = m_fault_msg.begin(); it != m_fault_msg.end(); ++it){
    msg = *it;
    var_name = toupper(biteStringX(msg, '='));
    if (var_name == "")
      return(false);
    
    ok = setDoubleOnString(dbl_val, msg);
    if (ok) {
      Notify(var_name, dbl_val);
    } else {
      // send as a string
      Notify(var_name, msg);
    }
  }

  m_sent_fault = true; 
  return(true);  
}


bool FalconRunMgr::sendRadChangeMsg()
{
  std::set<std::string>::iterator it;
  std::string msg;
  std::string var_name;
  
  for (it = m_rad_change_msg.begin(); it != m_rad_change_msg.end(); ++it){
    msg = *it;
    var_name = toupper(biteStringX(msg, '='));
    if ( (var_name != "") && (msg != "") ){
      Notify(var_name, msg);
    } else {
      return(false);
    }
  }

  m_sent_rad_change = true; 
  return(true);  
}


bool FalconRunMgr::handleLoopID(std::string val){
  
  std::string str = biteStringX(val, '=');
  double dbl_val;
  
  if ( (str == "id") &&  setDoubleOnString(dbl_val, val) ){
    m_curr_loop_id = static_cast<int>(dbl_val);
  } else {
    return(false);
  }

  if ( (m_curr_loop_id + 1 ) == m_loops_before_change_rad ) {
    // send the rad change message after this
    m_time_rad_flag_rcv = MOOSTime(); 
  }
  
  return(true); 
}
