/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: MultiAgentColFilt.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "MultiAgentColFilt.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

MultiAgentColFilt::MultiAgentColFilt()
{
  m_nav_x  = 0.0;
  m_nav_y  = 0.0;
  m_nav_hdg = 0.0;
  m_nav_spd = 0.0;
  m_vname = "";

  m_nav_x_last_rcv_time = 0.0;
  m_nav_y_last_rcv_time  = 0.0;
  m_nav_hdg_last_rcv_time = 0.0;
  m_nav_spd_last_rcv_time = 0.0;
   
  m_own_state_stale_thresh = 1.0;
  m_stale_node_rec_thresh  = 2.0;

  m_verbose = false;
  m_post_moos_manual_override = true;
  m_post_rev_thrust = false;

  m_emergency_rev_thrust_cmd = -70.0;

   
  m_emergency_posted = false;
}

//---------------------------------------------------------
// Destructor

MultiAgentColFilt::~MultiAgentColFilt()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool MultiAgentColFilt::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL")) {
      if ( !handleNodeReport(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "NAV_X") {
      m_nav_x = msg.GetDouble();
      m_nav_x_last_rcv_time = msg.GetTime(); 

    } else if (key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
      m_nav_y_last_rcv_time = msg.GetTime(); 

    } else if (key == "NAV_HEADING") {
      m_nav_hdg = msg.GetDouble();
      m_nav_hdg_last_rcv_time = msg.GetTime(); 
      
    } else if (key == "NAV_SPEED") {
      m_nav_spd = msg.GetDouble();
      m_nav_spd_last_rcv_time = msg.GetTime(); 

    } else if (key == "CONTACTS_LIST") {
      if ( !handleContactsList(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "DEPLOY") {
      if ( !setBooleanOnString(m_deployed, msg.GetString() ) )
	reportRunWarning("Unhandled Mail: " + key);

    }else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool MultiAgentColFilt::OnConnectToServer()
{
  //registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool MultiAgentColFilt::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if (!checkReadyToGo()){
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // update own reachable set
  bool ok = m_own_fwd_reach_set.updateSet(m_nav_x, m_nav_y,
					  m_nav_hdg, m_nav_spd, MOOSTime());

  if (!ok){
    reportRunWarning("Error updating own reachable set");
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // update reachable set for all contacts
  updateContactFwdReachSets();

  // check if own reachable set intersects with others
  std::string fwd_set_spec = "";
  if( checkFwdReachSetIntersects()){
    
    // we got problems!  Post as configured
    if (m_post_moos_manual_override)
      Notify("MOOS_MANUAL_OVERIDE","true");

    if (m_post_rev_thrust)
      Notify("DESIRED_THRUST", m_emergency_rev_thrust_cmd); 

    if (!m_emergency_posted){
      Notify("EMERGENCY_STOP_COMMANDED", m_emergency_stop_contact_name);
      m_estop_record[MOOSTime()] = m_emergency_stop_contact_name; 
    }
    m_emergency_posted = true;
    
    // post visuals if valid
    if (m_own_fwd_reach_set.isSpecValid()){
      fwd_set_spec = m_own_fwd_reach_set.getSpec("red");
      Notify("VIEW_POLYGON", fwd_set_spec);
    }

  } else {
    // Is this the first good posting after we detected
    // an intersection?
    if ( m_emergency_posted == true){

      // clear postings as configured
      if (m_post_moos_manual_override)
	Notify("MOOS_MANUAL_OVERIDE","false");
      
      m_emergency_posted = false;

      fwd_set_spec = m_own_fwd_reach_set.getSpecInactive();
      Notify("VIEW_POLYGON", fwd_set_spec); 
    }

    // otherwise post the set if desired
    if (m_verbose){
      if (m_own_fwd_reach_set.isSpecValid()){
	fwd_set_spec = m_own_fwd_reach_set.getSpec("green");
	Notify("VIEW_POLYGON", fwd_set_spec);
      }
      
    }
  }
  
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool MultiAgentColFilt::OnStartUp()
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
    if (param == "verbose_visuals"){
      handled = setBooleanOnString(m_verbose, value);
    } else if (param == "post_moos_manual_override"){
      handled = setBooleanOnString(m_post_moos_manual_override, value);
    } else if (param == "post_rev_thrust"){
      handled = setBooleanOnString(m_post_rev_thrust, value);
    } else if (param == "emergency_rev_thrust_cmd"){
      handled = setDoubleOnString(m_emergency_rev_thrust_cmd, value);
    } else if (param == "own_state_stale_thresh"){
      handled = setPosDoubleOnString(m_own_state_stale_thresh, value);
    } else if (param == "stale_node_rec_thresh"){
      handled = setPosDoubleOnString(m_stale_node_rec_thresh, value);
    } else if (param == "nominal_fwd_speed"){
      double temp_val;
      handled = setPosDoubleOnString(temp_val, value);
      if (handled)
	handled = m_own_fwd_reach_set.setNominalFwdSetSpeed(temp_val);
      if (handled)
	m_nominal_fwd_speed = temp_val;
      
    }else if(param == "nominal_fwd_reachable_set") {
      handled = m_own_fwd_reach_set.setNominalFwdSetPoly(value);
      if (handled)
	m_nominal_fwd_poly_spec = value;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  // The vehicle name is the host community
  m_vname = m_host_community;
  m_own_fwd_reach_set.setName(m_vname); 
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void MultiAgentColFilt::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
   Register("NODE_REPORT_LOCAL", 0);
   Register("CONTACTS_LIST", 0);
   Register("NAV_X", 0);
   Register("NAV_Y", 0);
   Register("NAV_HEADING", 0);
   Register("NAV_SPEED", 0);
   Register("DEPLOY", 0);
}


//------------------------------------------------------------
// Procedure: checkReadyToGo()
//             returns true if all good to go

bool MultiAgentColFilt::checkReadyToGo() 
{

  if (!m_deployed)
    return(false);

  if ((MOOSTime() - m_nav_x_last_rcv_time) > m_own_state_stale_thresh)
    return(false);
  if ((MOOSTime() - m_nav_y_last_rcv_time) > m_own_state_stale_thresh)
    return(false);
  if ((MOOSTime() - m_nav_hdg_last_rcv_time) > m_own_state_stale_thresh)
    return(false);
  if ((MOOSTime() - m_nav_spd_last_rcv_time) > m_own_state_stale_thresh)
    return(false);

  // don't bother if moving very slowly AND an emergeny was
  // not posted, this prevents an error from appearing on
  // pMarineViewer because the fwd reach polygon has an
  // area of zero due to precision.
  if ( (m_nav_spd < 0.05) && (!m_emergency_posted) )
    return(false); 
  
  return(true); 
}

//------------------------------------------------------------
// Procedure: updateContactFwdReachSets()
//            

void MultiAgentColFilt::updateContactFwdReachSets() 
{

  m_contacts_with_issues.clear();
  std::set<std::string>::iterator it;

  for (it = m_contacts.begin(); it != m_contacts.end(); it++){
    
    // is there a node report?
    if (m_node_rec_map.count(*it) == 0){
      m_contacts_with_issues.insert(*it); 
      continue;
    }
    
    // is it fresh?
    double time_lag = MOOSTime() - m_node_rec_map[*it].getTimeStamp();
    if (time_lag > m_stale_node_rec_thresh){
      m_contacts_with_issues.insert(*it); 
      continue;
    }

    // does it have all the info?
    bool setX = m_node_rec_map[*it].isSetX();
    bool setY = m_node_rec_map[*it].isSetY();
    bool setHdg = m_node_rec_map[*it].isSetHeading();
    bool setSpd = m_node_rec_map[*it].isSetSpeed();
    bool setTime = m_node_rec_map[*it].isSetTimeStamp();

    if ( !( setX && setY && setHdg && setSpd && setTime) ){
      m_contacts_with_issues.insert(*it); 
      continue;
    }
    
    // get all the info
    double c_x = m_node_rec_map[*it].getX();
    double c_y = m_node_rec_map[*it].getY();
    double c_hdg = m_node_rec_map[*it].getHeading();
    double c_spd = m_node_rec_map[*it].getSpeed();
    double c_time = m_node_rec_map[*it].getTimeStamp();

    
    // do we already have a fwd set in the map?
    if (m_contacts_fwd_reach_set.count(*it) == 0){
      // make a new one
      // set up the record
      ForwardSetEst newForwardSet;
      newForwardSet.setNominalFwdSetPoly(m_nominal_fwd_poly_spec);
      newForwardSet.setNominalFwdSetSpeed(m_nominal_fwd_speed); ;
      newForwardSet.setName(*it);

      // update it with current info
      newForwardSet.updateSet(c_x, c_y, c_hdg, c_spd, c_time);

      // add it to the map
      m_contacts_fwd_reach_set[*it] = newForwardSet; 

    } else {
      // update the exisiting one
      m_contacts_fwd_reach_set[*it].updateSet(c_x, c_y, c_hdg, c_spd, c_time); 
    }
      
  }
  
  // all up to date!
  return; 
}


//------------------------------------------------------------
// Procedure: checkFwdReachSetIntersects()
//            Assumes the reachable sets have been updated!!!!

bool MultiAgentColFilt::checkFwdReachSetIntersects() 
{
  std::set<std::string>::iterator it;
  XYPolygon own_curr_reach_set = m_own_fwd_reach_set.getCurrFwdReachSet();

  for (it = m_contacts.begin(); it != m_contacts.end(); it++){
    
    if (m_contacts_with_issues.count(*it) > 0)
      continue;

    if (m_contacts_fwd_reach_set[*it].willCollide(own_curr_reach_set)){
      m_emergency_stop_contact_name = *it;
      return(true);

    }
  }
 
  return(false); 
}

//------------------------------------------------------------
// Procedure: buildReport()

bool MultiAgentColFilt::buildReport() 
{
  m_msgs << "============================================" << endl;
  if (!m_emergency_posted){
    m_msgs << "    Running:  Currently All Safe          " << endl;

  } else {
    m_msgs << "  !Emergency Stop for " << m_emergency_stop_contact_name <<"!     " << endl;
  }
  m_msgs << "============================================" << endl;
  m_msgs << "                                            " << endl;
  m_msgs << "                                            " << endl;
  m_msgs << "Emergency stop history:                     " << endl;
  m_msgs << "============================================" << endl;
  
  unsigned int count = 1;
  std::map<double, std::string>::iterator it;
  for (it=m_estop_record.begin(); it!=m_estop_record.end(); it++){
    m_msgs << "  [" << uintToString(count) << "] ";
    m_msgs << "contact: " << it->second << ", ";
    m_msgs << "time: " << doubleToStringX(it->first,2) << " ";
    m_msgs << "(" << doubleToStringX((MOOSTime() - it->first),2) << " sec ago)";
    m_msgs << endl;
    count += 1;
  }

 

  return(true);
}




 
//-----------------------------------------------------
// Handle Node Report
//
bool MultiAgentColFilt::handleNodeReport(std::string msg)
{
 
  // process incoming node record 
  NodeRecord newNodeRecord;
  newNodeRecord = string2NodeRecord(msg);
  if (!newNodeRecord.valid())
    return(false);

  // Only keep contacts that are alive:
  if ( m_contacts.count(newNodeRecord.getName()) || (newNodeRecord.getName() == m_vname) )
    m_node_rec_map[newNodeRecord.getName()] = newNodeRecord;
  
  return(true);
 }



//-----------------------------------------------------
// Handle Contact List Report
//
 bool MultiAgentColFilt::handleContactsList(std::string msg)
{
  // parse message
  std::set<std::string> new_contact_set;
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i=0; i<svector.size(); i++) {
    new_contact_set.insert(svector[i]);
  }
  m_contacts = new_contact_set;

  // Now clear out any old node reports
  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end();){
    if ( m_contacts.count(it->first) || (m_vname == it->first) ) {
	++it;
      } else {
	m_node_rec_map.erase(it++);
      }
  }
 
  return(true);
 }
