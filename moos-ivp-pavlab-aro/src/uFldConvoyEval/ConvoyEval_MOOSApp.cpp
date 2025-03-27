/************************************************************/
/*    NAMES: Tyler Paine, Michael R. Benjamin               */      
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: FldConvoyEval.cpp                               */
/*    DATE: June 5th, 2022                                  */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ConvoyEval_MOOSApp.h"
#include "NodeMessage.h"
#include "NodeMessageUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ConvoyEval_MOOSApp::ConvoyEval_MOOSApp()
{
  m_prev_max_pairwise_range = 0;
  m_ever_max_pairwise_range = 0;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ConvoyEval_MOOSApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  string recap_var = m_fld_engine.getStrString("recap_var");
  string stat_recap_var = m_fld_engine.getStrString("stat_recap_var");
  string spd_policy_var = m_fld_engine.getStrString("spd_policy_var");

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval   = msg.GetString(); 

    
#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval   = msg.GetDouble();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    bool handled;
    if(key == stat_recap_var) 
      handled = m_fld_engine.handleStatRecap(sval);
    else if(key == recap_var) 
      handled = m_fld_engine.handleRecap(sval);
    else if(key == spd_policy_var) 
      handled = m_fld_engine.handleSpdPolicy(sval);
    else if (key == "NODE_REPORT")
      handled = m_fld_engine.handleNodeReport(sval);
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);

    if(!handled)
      reportRunWarning("Mail Not Handled: " + key);
    
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ConvoyEval_MOOSApp::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool ConvoyEval_MOOSApp::Iterate()
{
  AppCastingMOOSApp::Iterate();

  m_fld_engine.setCurrTime(m_curr_time);
  m_fld_engine.updateMetrics();

  Notify("CONVOY_LENGTH", m_fld_engine.getDouble("convoy_length"));
  
  int max_pairwise_range = (int)(m_fld_engine.getMaxPairwiseRange());
  if(max_pairwise_range != m_prev_max_pairwise_range)
    Notify("CONVOY_MAX_RNG", max_pairwise_range);
  if(max_pairwise_range > m_ever_max_pairwise_range) {
    Notify("CONVOY_MAX_RNG_EVER", max_pairwise_range);
    m_ever_max_pairwise_range = max_pairwise_range;
  }
    

  m_prev_max_pairwise_range = max_pairwise_range;

  string convoy_order = m_fld_engine.getConvoyReport();
  if(convoy_order != m_prev_convoy_order)
    Notify("CONVOY_ORDER", convoy_order);
  m_prev_convoy_order = convoy_order;
      
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool ConvoyEval_MOOSApp::OnStartUp()
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


    bool handled = m_fld_engine.setParam(param, value);
    
    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  m_fld_engine.setCurrTime(m_curr_time);
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ConvoyEval_MOOSApp::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  string recap_var = m_fld_engine.getStrString("recap_var");
  string stat_recap_var = m_fld_engine.getStrString("stat_recap_var");
  string spd_policy_var = m_fld_engine.getStrString("spd_policy_var");

  if(isAlphaNum(recap_var, "_"))
    Register(recap_var, 0);
  if(isAlphaNum(stat_recap_var, "_"))
    Register(stat_recap_var, 0);
  if(isAlphaNum(spd_policy_var, "_"))
    Register(spd_policy_var, 0);

  Register("NODE_REPORT", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool ConvoyEval_MOOSApp::buildReport() 
{
  vector<string> msgs = m_fld_engine.buildReport();
  for(unsigned int i=0; i<msgs.size(); i++)
    m_msgs << msgs[i] << endl;
  return(true);
}
