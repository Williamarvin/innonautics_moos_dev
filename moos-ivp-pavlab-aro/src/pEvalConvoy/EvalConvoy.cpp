/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: EvalConvoy.cpp                                       */
/*    DATE: July 11th, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "EvalConvoy.h"
#include "NodeMessage.h"
#include "NodeRecord.h"
#include "NodeMessageUtils.h"
#include "NodeRecordUtils.h"

using namespace std;

//---------------------------------------------------------
// Procedure: OnNewMail

bool EvalConvoy::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  string recap_var = m_engine.getStrString("recap_var");
  string stat_recap_var = m_engine.getStrString("stat_recap_var");
  string spd_policy_var = m_engine.getStrString("spd_policy_var");

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval  = msg.GetString(); 

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
#endif

    bool   handled = true;
    string whynot;
    
    if(key == stat_recap_var) 
      m_engine.handleStatRecap(sval);
    else if(key == recap_var) 
      m_engine.handleRecap(sval);
    else if(key == spd_policy_var) 
      m_engine.handleSpdPolicy(sval);
    else if(key == "CONVOY_STAT_RECAP_ALLY") 
      m_engine.handleStatRecapAlly(sval);
    else if(key == "NODE_REPORT_LOCAL_FIRST") 
      handled = handleMailNodeReport(sval, whynot);
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      handled = false;

    if(!handled) {
      string warning = "Unhandled Mail: " + key;
      if(whynot != "")
	warning += " " + whynot;
      reportRunWarning(warning);
    }    
  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool EvalConvoy::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool EvalConvoy::Iterate()
{
  AppCastingMOOSApp::Iterate();

  m_engine.setCurrTime(m_curr_time);
  m_engine.updateMetrics();

  Notify("CONVOY_RANGE", m_engine.getDouble("convoy_rng"));

  shareStatRecap();
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool EvalConvoy::OnStartUp()
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

    bool handled = m_engine.setParam(param, value);

    if(param == "match_group")
      handled = setBooleanOnString(m_match_group, value);
    
    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  m_engine.setCurrTime(m_curr_time);
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void EvalConvoy::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  string recap_var = m_engine.getStrString("recap_var");
  string stat_recap_var = m_engine.getStrString("stat_recap_var");
  string spd_policy_var = m_engine.getStrString("spd_policy_var");

  if(isAlphaNum(recap_var, "_"))
    Register(recap_var, 0);
  if(isAlphaNum(stat_recap_var, "_"))
    Register(stat_recap_var, 0);
  if(isAlphaNum(spd_policy_var, "_"))
    Register(spd_policy_var, 0);

  Register("CONVOY_STAT_RECAP_ALLY", 0);
  Register("NODE_REPORT_LOCAL_FIRST", 0);
}

//------------------------------------------------------------
// Procedure: handleMailNodeReport()

bool EvalConvoy::handleMailNodeReport(string str, string& whynot)
{
  NodeRecord new_record = string2NodeRecord(str);
  if(!new_record.valid("x,y,name", whynot))
    return(false);
  
  string group_name = new_record.getGroup();

  if(group_name != "") {
    if((m_group_name != "") && (group_name != m_group_name)) {
      whynot = "detected group name does not match declared group name";
      return(false);
    }
    m_group_name = group_name;
  }

  return(true);
}


//---------------------------------------------------------
// Procedure: buildReport()

bool EvalConvoy::buildReport() 
{
  m_msgs << "Configuration: " << endl;
  m_msgs << "  match_group: " << boolToString(m_match_group) << endl;
  m_msgs << "   group_name: " << m_group_name << endl;
  
  vector<string> msgs = m_engine.buildReport();
  for(unsigned int i=0; i<msgs.size(); i++)
    m_msgs << msgs[i] << endl;
  return(true);
}


//---------------------------------------------------------
// Procedure: shareStateRecap()
//   Purpose: Share the CONVOY_STAT_RECAP info with other
//            vehicles. Share once every ten seconds, unless
//            a change has been detected between successive
//            stat recaps.

void EvalConvoy::shareStatRecap()
{
  string stat_recap = m_engine.getStatRecapSpec();
  if(stat_recap == "")
    return;

  if(m_last_stat_recap == stat_recap) {
    double elapsed = m_curr_time - m_last_share_tstamp;
    if(elapsed < 30)
      return;
  }

  m_last_stat_recap = stat_recap;
  m_last_share_tstamp = m_curr_time;

  NodeMessage node_message;
  node_message.setSourceNode(m_host_community);
  node_message.setVarName("CONVOY_STAT_RECAP_ALLY");
  node_message.setStringVal(stat_recap);
  node_message.setColor("invisible");

  node_message.setSourceApp("pEvalConvoy");
  //node_message.setMessageID(uintToString(m_iteration));

  
  if(m_match_group && (m_group_name != ""))
    node_message.setDestGroup(m_group_name);
  else
    node_message.setDestNode("all");
    
  string msg = node_message.getSpec();

  Notify("NODE_MESSAGE_LOCAL", msg);		 
}
