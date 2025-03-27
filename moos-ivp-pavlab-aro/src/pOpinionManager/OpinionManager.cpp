/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionManager.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "OpinionManager.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

OpinionManager::OpinionManager()
{
  m_vname = "";

  m_optn_node_message.setVarName("OPINION_MESSAGE");
  m_optn_node_message.setColor("off");
  m_optn_node_message.setSourceApp("OpinionManager");
  m_optn_node_message.setDestNode("all");
}

//---------------------------------------------------------
// Destructor

OpinionManager::~OpinionManager()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool OpinionManager::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval   = msg.GetString();
    double dval   = msg.GetDouble();
    double mtime  = msg.GetTime();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();

    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    //bool reset_requested = false;
    if(key == "OPINION_MESSAGE") {
      //if (!reset_requested)
      m_op_eng.receiveOpinionMessage(sval, mtime);
      
    } else if (key == "PROPOSAL_MESSAGE") {

    } else if (m_op_eng.isValInBuffer(key)) {
      if (msg.IsDouble()){
	m_op_eng.loadValIntoBuffer(key, dval);
      } else if (msg.IsString()){
	m_op_eng.loadValIntoBuffer(key, sval);
      }
    
    } else if (key == "RESET_OPINIONS"){
      m_op_eng.resetOpinionState(MOOSTime(), 5.0);
      m_last_option_active_posting.clear();
      //reset_requested = true; 
      
    } else if(key != "APPCAST_REQ") {// handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
    
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool OpinionManager::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool OpinionManager::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Check if ready, and bail if not
  if ( !m_op_eng.readyToCalculate(MOOSTime() ) ){
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // Check if any options are active, bail if none are active
  int numb_active = m_op_eng.numberOptionsActive();
  if (numb_active == 0){
    AppCastingMOOSApp::PostReport();
    return(true);
  }
  
  // Flags for each stage
  bool iterate_successful = false;
  bool ok_to_publish = false;
  bool message_ok = false;
  bool ok_to_post_lists = false;
  
  // Iterate opinion
  iterate_successful = m_op_eng.iterateOwnOpinion( MOOSTime() );
  
  // Publish the outputs associated with the dominant
  // opinion
  std::vector<std::string> var_names;
  std::vector<std::string> values; 
  ok_to_publish = m_op_eng.getOutputsForLargestOpinion(var_names, values);
  
  // Publish outputs if all good, and we have not posted these before
  if (iterate_successful && ok_to_publish) {
    for (unsigned int i = 0; i< var_names.size(); i++){
      bool post_outputs = true; 
      if (m_last_option_active_posting.count(var_names[i]) > 0) {
	if ( m_last_option_active_posting[var_names[i]] == values[i] )  {
	  post_outputs = false;   // Key found and the value was the same. 
	}
      }
      if (post_outputs){
	Notify(var_names[i], values[i]);
	m_last_option_active_posting[var_names[i]] = values[i];
      }
      
    }
  }
  
  // Get own opinion to send to others
  std::string optn_msg;
  message_ok = m_op_eng.buildOwnOpinionMessage(optn_msg);
  
  // Send out if all good
  if (iterate_successful && message_ok){
    
    m_optn_node_message.setStringVal(optn_msg);
    string msg = m_optn_node_message.getSpec(); 
    Notify("NODE_MESSAGE_LOCAL", msg);
    
    // For own log file
    Notify("OWN_OPINION_STATE", optn_msg);
    Notify("OWN_ATTENTION", m_op_eng.getAttentionU()); 
    
  }

  // Finally, post lists of neighbors status
  std::vector<std::string> opt_names;
  std::vector<std::string> agent_list;
  ok_to_post_lists = m_op_eng.getUpdatedOptionStateLists(opt_names, agent_list, MOOSTime() );
  
  // Publish outputs if all good, and these are new postings
  if (ok_to_post_lists) {
    for (unsigned int i = 0; i< opt_names.size(); i++){

      bool post = true; 
      if (m_last_population_state_posting.count(opt_names[i]) > 0) {
	if ( m_last_population_state_posting[opt_names[i]] == agent_list[i] )  {
	  post = false;   // Key found and the value was the same. 
	}
      }

      if ( post ) {
	Notify(toupper(opt_names[i] + "_LIST"), agent_list[i]); 
	m_last_population_state_posting[opt_names[i]] = agent_list[i];
      } 
    }
  }
  
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool OpinionManager::OnStartUp()
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
    std::string extra_warnings = "";

    if(param == "node_message_color") {
      // don't check for valid color since NodeMessage just
      // uses the default if invalid color is provided
      if (value != ""){
	 m_optn_node_message.setColor(value);
	 handled = true; 
      } else {
	handled = false; 
      }
    }

    else if(param == "node_message_dest_group"){
      if (value != ""){
	m_optn_node_message.setDestGroup(value);
	m_optn_node_message.setDestNode("");
	handled = true; 
      } else {
	handled = false; 
      }
    }  
    else if(param == "tau_u") {
      double temp;
      handled =  setPosDoubleOnString(temp,value);
      if(handled) {
	handled = m_op_eng.setTauU(temp);
      }
    }
    
    else if(param == "min_attention") {
      double temp;
      handled =  setPosDoubleOnString(temp,value);
      if(handled) {
	handled = m_op_eng.setMinAtten(temp);
      }
    }

    else if(param == "max_attention") {
      double temp;
      handled =  setPosDoubleOnString(temp,value);
      if(handled) {
	handled = m_op_eng.setMaxAtten(temp);
      }
    }
    

    else if(param == "u_th") {
      double temp;
      handled =  setPosDoubleOnString(temp,value);
      if(handled) {
	handled = m_op_eng.setUth(temp);
      }
    }
	
    else if(param == "satfunorder") {
      double temp;
      handled =  setPosDoubleOnString(temp,value);
      if(handled) {
	handled = m_op_eng.setSatFunOrder(temp);
      }
    }

    else if(param == "opinion_thresh") {
      double temp;
      handled =  setPosDoubleOnString(temp,value);
      if(handled) {
	handled = m_op_eng.setOpinionThresh(temp);
      }
    }
    
    else if(param == "neighbor_msg_stale_thresh") {
      double temp;
      handled =  setPosDoubleOnString(temp,value);
      if(handled) {
	handled = m_op_eng.setStaleThresh(temp);
      }
    }
    else if(param == "numerical_int_gain") {
      double temp;
      handled =  setPosDoubleOnString(temp,value);
      if(handled) {
	handled = m_op_eng.setNumIntGain(temp);
      }
    }
    
    else if(param == "option_file") {
      handled = m_op_eng.handleOptConfig(value, extra_warnings);
    }
    
    else if(param == "group") {
      handled = m_op_eng.setGroup(value);
    }
    else if(param == "socialtick") {
      double temp;
      handled =  setPosDoubleOnString(temp,value);
      if(handled) {
	handled = m_op_eng.setSocialTick(temp);
      }
    }

    if(!handled)
      reportUnhandledConfigWarning(orig + " " + extra_warnings);

  }

  

  // Set up the config engine
  bool vnameOk = m_MissionReader.GetValue("Community", m_vname);
  bool ok1 = false;
  bool ok2 = false;
  if (vnameOk) {
    ok1 = m_op_eng.setName(m_vname);
    ok2 = m_op_eng.setupOpinionManager(MOOSTime());
    // Set node message here
    m_optn_node_message.setSourceNode(m_vname);
  }

  if( !(ok1 && ok2) ) {
    reportUnhandledConfigWarning("OpinionManagerEngine setup was not successful");
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void OpinionManager::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("OPINION_MESSAGE", 0);
  Register("PROPOSAL_MESSAGE",0);
  Register("RESET_OPINIONS",0); 

  // The OpinionManagerEngine was just set up
  // Register for all the vars as needed
  std::vector<std::string> vars_to_register;
  vars_to_register = m_op_eng.getVarsToRegister();

  for (unsigned int i = 0; i < vars_to_register.size(); i++){
    Register(vars_to_register[i], 0);
  }
							  
  
}


//------------------------------------------------------------
// Procedure: buildReport()

bool OpinionManager::buildReport() 
{
  /*
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();
  */

  list<string> report_lines = m_op_eng.getReport(MOOSTime());
  list<string>::iterator p;
  for(p=report_lines.begin(); p!=report_lines.end(); p++) {
    string line = *p;
    m_msgs << line << endl;
  }

  return(true);
}




