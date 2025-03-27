/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: HVUProtectAssetCtrl.cpp                         */
/*    DATE: May 2024                                        */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "HVUProtectAssetCtrl.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

HVUProtectAssetCtrl::HVUProtectAssetCtrl()
{
  m_post_fake_hvu_node_report        = false;
  m_post_fake_hvu_node_report_to_all = false;
  
  m_last_fake_node_report_post_time = 0.0;
  m_hvu_name = "rex";
  
  m_number_of_tasks_sent = 0;
  m_last_time_tasks_sent = 0.0;
  m_task_resend_interval = 20.0; 
  
  m_region_update_thresh_dist = 10.0;

  m_cumulative_rad_change     = 0.0;
  m_add_noise = false;
  
}

//---------------------------------------------------------
// Destructor

HVUProtectAssetCtrl::~HVUProtectAssetCtrl()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool HVUProtectAssetCtrl::OnNewMail(MOOSMSG_LIST &NewMail)
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
      bool ok = handleNodeReport(msg.GetString());
      if (!ok)
	reportRunWarning("Unhandled Mail: " + key);

    } else if (key == "ZONE_UP_RAD"){
      double up_val = msg.GetDouble(); 
      m_pursuit_trigger_poly.grow_by_amt(up_val);
      m_cumulative_rad_change += up_val;

      m_pursuit_trigger_poly.set_color("vertex", "yellow");
      m_pursuit_trigger_poly.set_color("edge", "red");
      Notify("VIEW_POLYGON", m_pursuit_trigger_poly.get_spec());

    } else if (key == "ZONE_UP_RAD_RESET"){
      m_pursuit_trigger_poly.grow_by_amt(-m_cumulative_rad_change);
      m_cumulative_rad_change = 0.0;

      m_pursuit_trigger_poly.set_color("vertex", "yellow");
      m_pursuit_trigger_poly.set_color("edge", "red");
      Notify("VIEW_POLYGON", m_pursuit_trigger_poly.get_spec());
      
      
    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
  }
  
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool HVUProtectAssetCtrl::OnConnectToServer()
{
  //registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool HVUProtectAssetCtrl::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // First check if any intruders are within the configured
  // Trigger region
  // Returns true if any intruders are detected, and
  // m_names_active_intruders will be updated
  
  bool intruders_to_intercept = checkForIntruders();

  // Now check if we need to send tasks. 
  if (intruders_to_intercept){
    taskNodesToIntercept();
  }

  // Finally clean-up the book-keeping when an intruder
  // is detected, a task was sent, and now the intruder has
  // left the trigger region.
  
  std::set<std::string>::iterator it;
  for (it = m_names_task_sent.begin(); it != m_names_task_sent.end();){
    if (m_names_active_intruders.count(*it)){
      // still active
      it++;
    } else {
      // no longer active, and task was sent
      taskNodesToStopIntercept(*it);
      // and erase it.
      m_names_task_sent.erase(it++);
     
    }
  }
  

  // Post fake HVU node report if desired.
  // This is mostly for debugging. 
  if ( (m_post_fake_hvu_node_report) ){
    if((MOOSTime() -  m_last_fake_node_report_post_time) > 1.0 )
      postFakeHVUNodeReport();
  }
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool HVUProtectAssetCtrl::OnStartUp()
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
    if(param == "pursuit_trigger_region") {
      m_pursuit_trigger_poly = string2Poly(value);
      m_pursuit_trigger_poly.set_label("trigger_region");
      handled = m_pursuit_trigger_poly.is_convex();
      if (handled){
	m_pursuit_trigger_region_centriod = m_pursuit_trigger_poly.get_centroid_pt();
	m_pursuit_trigger_poly.set_color("vertex", "yellow");
	m_pursuit_trigger_poly.set_color("edge", "red");
	Notify("VIEW_POLYGON", m_pursuit_trigger_poly.get_spec());
      }

    }
    else if(param == "intruder_names") {
      handled = handleConfigIntruderNames(value);
    }
    else if(param == "hvu_name") {
      m_hvu_name = value; 
      handled = (value != "");
    }
    else if (param == "post_fake_hvu_node_report") {
      handled = setBooleanOnString(m_post_fake_hvu_node_report, value);
    }
    else if (param == "post_fake_hvu_node_report_to_all") {
      handled = setBooleanOnString(m_post_fake_hvu_node_report_to_all, value);
    }
    else if (param == "region_update_thresh_dist") {
      handled = setPosDoubleOnString(m_region_update_thresh_dist, value); 
    }
    else if (param == "add_position_noise"){
      handled = setBooleanOnString(m_add_noise, value); 
    }
    

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  GeodesySetup();
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void HVUProtectAssetCtrl::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
   Register("NODE_REPORT", 0);
   Register("NODE_REPORT_LOCAL", 0);
   Register("ZONE_UP_RAD",0);
   Register("ZONE_UP_RAD_RESET",0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool HVUProtectAssetCtrl::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}


//---------------------------------------------------------
// Procedure: GeodesySetup()
//   Purpose: Initialize geodesy object with lat/lon origin.
//            Used for LatLon2LocalUTM conversion.

bool HVUProtectAssetCtrl::GeodesySetup()
{
  double LatOrigin = 0.0;
  double LonOrigin = 0.0;

  // Get Latitude Origin from .MOOS Mission File
  bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
  if(!latOK) {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return(false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
  if(!lonOK){
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return(false);
  }

  // Initialise CMOOSGeodesy object
  bool geoOK = m_geodesy.Initialise(LatOrigin, LonOrigin);
  if(!geoOK) {
    reportConfigWarning("CMOOSGeodesy::Initialise() failed. Invalid origin.");
    return(false);
  }

  return(true);
}

//-----------------------------------------------------
// Handle Node Report
//
bool HVUProtectAssetCtrl::handleNodeReport(std::string msg)
{
  // process incoming node record 
  NodeRecord newNodeRecord;
  newNodeRecord = string2NodeRecord(msg, true);
  if (!newNodeRecord.valid())
    return(false);
  
  // check if name is in the set of intruders
  std::string rec_name = newNodeRecord.getName();
  if (m_intruder_names.count(rec_name) ) {
    m_intruder_node_rec_map[rec_name] = newNodeRecord; 
    return(true); 
  }

  // check if this is the hvu (which could be ownship)
  if (rec_name == m_hvu_name) {
    bool ok = possiblyUpdateNVPZ(newNodeRecord);
    if (!ok)
      return(false);

    if (m_post_fake_hvu_node_report_to_all)
      Notify("NODE_REPORT_ALL", msg);
  }

  return(true); 
}


//---------------------------------------------------------
// Procedure: handleConfigIntruderNames()
//  Examples: intruder_names = fin
//            intruder_names = abe, ben, cal

bool HVUProtectAssetCtrl::handleConfigIntruderNames(std::string str)
{
  bool all_ok = true;
  
  std::vector<std::string> names = parseString(str, ',');
  for(unsigned int i=0; i<names.size(); i++) {
    std::string name = stripBlankEnds(names[i]);
    if(name != "")
      m_intruder_names.insert(name);
    else
      all_ok = false;
  }

  return(all_ok);
}


//--------------------------------------------------
// Possibly update the naval vehicle protection zone
//
bool HVUProtectAssetCtrl::possiblyUpdateNVPZ(NodeRecord newNodeRecord)
{
  
   // Convert lat long to x y,
  // use lat long if set, otherwise resort to x y
  double node_x, node_y;
  
  if ( (newNodeRecord.isSetLatitude() && newNodeRecord.isSetLongitude() ) ){

    double dbl_lat = newNodeRecord.getLat();
    double dbl_lon = newNodeRecord.getLon();

    bool ok = m_geodesy.LatLong2LocalGrid(dbl_lat, dbl_lon, node_y, node_x);

    if (!ok)
      return(false);
    
  } else if ( (newNodeRecord.isSetX() && newNodeRecord.isSetY() ) ) {
    node_x = newNodeRecord.getX();
    node_y = newNodeRecord.getY(); 
      
  } else {
    return(false);
  }
  
  // Only update the region if the centriod has changed
  // significantly.  This prevents unnecessary updates
  // that are due to sensor noise. 
  double eps = 1.0; // meters
  double dist;
  dist= distPointToPoint( m_pursuit_trigger_region_centriod.get_vx(),
			  m_pursuit_trigger_region_centriod.get_vy(),
			  node_x, node_y);

  
  if  (dist > eps) {
    m_pursuit_trigger_poly.new_centroid(node_x, node_y);

    // also update the centriod points
    m_pursuit_trigger_region_centriod = m_pursuit_trigger_poly.get_centroid_pt();
    m_pursuit_trigger_poly.set_color("vertex", "yellow");
    m_pursuit_trigger_poly.set_color("edge", "red");
    
    Notify("VIEW_POLYGON", m_pursuit_trigger_poly.get_spec());
  }

  return(true); 
}



//-------------------------------------------------
// checkForIntruders()

//     First check if any intruders are within the configured
//     Trigger region
//     Returns true if any intruders are detected, and
//     m_names_active_intruders will be updated

bool HVUProtectAssetCtrl::checkForIntruders()
{
  std::set<std::string> new_list_of_active_intruders;

  double stale_thresh = 10.0; 

  // every node in this map is an intruder
  double x, y, rec_time;
  std::string vname; 
  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_intruder_node_rec_map.begin(); it != m_intruder_node_rec_map.end(); it++){
    x = it->second.getX();
    y = it->second.getY();
    rec_time = it->second.getTimeStamp(); 
    vname = it->second.getName();

    double dt = MOOSTime() - rec_time;
    if (dt < 0.0) { dt = -dt;};
    
    if (m_pursuit_trigger_poly.contains(x, y) && (dt < stale_thresh)) {
      new_list_of_active_intruders.insert(vname); 
    }

  }

  m_names_active_intruders = new_list_of_active_intruders;

  if (m_names_active_intruders.size() > 0)
    return(true); 
  
  return(false); 
};


//-------------------------------------------------
// taskNodesToIntercept()

//    Sends tasks to nodes when intercept is needed


void HVUProtectAssetCtrl::taskNodesToIntercept()
{
  // Have we already sent tasks for these intruders?
  // If not then send a new task.

  bool at_least_one_task_sent = false; 

  std::set<std::string>::iterator it;
  for (it = m_names_active_intruders.begin(); it != m_names_active_intruders.end(); it++){

    
    // Have we already sent tasks for this intruder?
    // And has enough time passed that we need to send again
    bool sent_this_task = m_names_task_sent.count(*it) > 0;
    bool enough_time_elapsed = (MOOSTime() - m_last_time_tasks_sent) > m_task_resend_interval; 
    if (sent_this_task && !enough_time_elapsed)
      continue;
    
    // sanity check record is in the map. 
    if (m_intruder_node_rec_map.count(*it) == 0) {
      continue; 
    }

    // Ok, we are ready to go
    double x, y;
    x = m_intruder_node_rec_map[*it].getX();
    y = m_intruder_node_rec_map[*it].getY();
    // record should be less than 10 seconds old.
    std::string intruder_name = m_intruder_node_rec_map[*it].getName();

    MissionTask new_mission_task;
    std::string mission_task_details = "intercept=" + intruder_name;
    mission_task_details += ",x=" + to_string(x);
    mission_task_details += ",y=" + to_string(y);

    new_mission_task.setTaskType("intercept");
    new_mission_task.setTaskID("intercept" + to_string(m_number_of_tasks_sent));
    new_mission_task.setTaskDetails(mission_task_details);
    new_mission_task.setTaskTime(MOOSTime());
    new_mission_task.setTaskExempt(m_hvu_name);
    
    std::set<std::string>::iterator it2;
    for (it2 = m_intruder_names.begin(); it2 != m_intruder_names.end(); it2++){
      new_mission_task.addTaskExempt(*it2); 
    }

    std::string task_desc = new_mission_task.getSpec();
    
    Notify("MISSION_TASK_ALL", task_desc);
    
    // also send out via node message if we are operating within
    // the community of the HVU
    NodeMessage node_message;
    node_message.setSourceNode(m_hvu_name);
    node_message.setDestGroup("protectors");
    //node_message.setDestNode("all");
    node_message.setVarName("MISSION_TASK");
    node_message.setStringVal(task_desc); 
    Notify("NODE_MESSAGE_LOCAL", node_message.getSpec());


    sendPulse("red");
    
    // Finally update the set of intruder names that have
    // already been tasked. 
    m_names_task_sent.insert(*it);
    m_number_of_tasks_sent++;

    at_least_one_task_sent = true;
    
  }

  if (at_least_one_task_sent)
    m_last_time_tasks_sent = MOOSTime(); 
  
  return;
}

//-------------------------------------------------
// taskNodesToStopIntercept()

//    Sends tasks to nodes when intercept is no longer needed


void HVUProtectAssetCtrl::taskNodesToStopIntercept(std::string intruder_name)
{

  MissionTask new_mission_task;
  std::string mission_task_details = "stop_intercept=" + intruder_name;
  
  new_mission_task.setTaskType("intercept");
  new_mission_task.setTaskID("intercept" + to_string(m_number_of_tasks_sent));
  new_mission_task.setTaskDetails(mission_task_details);
  new_mission_task.setTaskTime(MOOSTime());
  new_mission_task.setTaskExempt(m_hvu_name);
  
  std::set<std::string>::iterator it2;
  for (it2 = m_intruder_names.begin(); it2 != m_intruder_names.end(); it2++){
    new_mission_task.addTaskExempt(*it2); 
  }
  
  std::string task_desc = new_mission_task.getSpec();
  
  Notify("MISSION_TASK_ALL", task_desc);
  
  // also send out via node message if we are operating within
  // the community of the HVU
  NodeMessage node_message;
  node_message.setSourceNode(m_hvu_name);
  node_message.setDestGroup("protectors");
  //node_message.setDestNode("all");
  node_message.setVarName("MISSION_TASK");
  node_message.setStringVal(task_desc); 
  Notify("NODE_MESSAGE_LOCAL", node_message.getSpec());

  sendPulse("green");
  
  m_number_of_tasks_sent++;
  
  return;
}

  


//--------------------------------------------------
// Post fake HVU data
//
bool HVUProtectAssetCtrl::postFakeHVUNodeReport()
{
  
  double dbl_lat;
  double dbl_lon;
  double node_x = m_pursuit_trigger_region_centriod.get_vx() + 0.0;
  double node_y = m_pursuit_trigger_region_centriod.get_vy() + 0.0;

  // generate random noise
  unsigned long int tseed = time(NULL);
  unsigned long int pseed = getpid() + 1;
  
  unsigned int rseed = (tseed*pseed) % 50000;
  std::default_random_engine generator (rseed);
  
  std::normal_distribution<double> distribution (0.0,0.5);
  double noise_x = distribution(generator);
  double noise_y = distribution(generator);

  if (m_add_noise){
    node_x = node_x + noise_x;
    node_y = node_y + noise_y;
  }
  
  bool ok = m_geodesy.LocalGrid2LatLong(node_x, node_y, dbl_lat, dbl_lon);
  
  NodeRecord newFakeRecord;
  newFakeRecord.setName(m_hvu_name);
  newFakeRecord.setType("ship");
  newFakeRecord.setX( node_x );
  newFakeRecord.setY( node_y );
  newFakeRecord.setLat( dbl_lat);
  newFakeRecord.setLon( dbl_lon);
  newFakeRecord.setSpeed(0.0);
  newFakeRecord.setHeading(64.0);
  newFakeRecord.setTimeStamp(MOOSTime());
  //newFakeRecord.setGroup("blue_team");

  if (m_post_fake_hvu_node_report)
    Notify("NODE_REPORT", newFakeRecord.getSpec());

  m_last_fake_node_report_post_time = MOOSTime();

  return(true);

}


//--------------------------------------------------------
// Procedure: sendPulse()
void HVUProtectAssetCtrl::sendPulse(std::string color) {

  double node_x = m_pursuit_trigger_region_centriod.get_vx() + 0.0;
  double node_y = m_pursuit_trigger_region_centriod.get_vy() + 0.0;

  double rad = m_pursuit_trigger_poly.max_radius(); 
  
  XYRangePulse pulse;
  pulse.set_x(node_x);                
  pulse.set_y(node_y);                
  pulse.set_label("mission_task_pulse");
  pulse.set_rad(rad);
  pulse.set_time(MOOSTime());       
  pulse.set_color("edge", "yellow");
  pulse.set_color("fill", color);
  pulse.set_duration(4.0);

  string spec = pulse.get_spec();
  Notify("VIEW_RANGE_PULSE", spec);
  return; 
}
