/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: HVUProtectNodeCtrl.cpp                          */
/*    DATE: May 2024                                        */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "HVUProtectNodeCtrl.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

HVUProtectNodeCtrl::HVUProtectNodeCtrl()
{

  m_nav_x = -1;
  m_nav_y = -1;
  m_odometry = 0.0;

  m_send_new_ignore_set = false;
  m_cur_option = "";
  m_inside_zone_list = "";
  m_outside_zone_list = "";

  m_hvu_name ="";
  m_closest_intruder_name = ""; 

  m_min_batt = 11.5;
  m_max_batt = 15.5;
  m_m300_batt_voltage = 0;
  m_got_real_batt_voltage = false;
  m_current_batt_voltage = 0.0;
  m_current_batt_voltage_filtered = 0.0; 
  m_time_last_got_batt = MOOSTime();


  m_intercept_input_range_of_max_val = 10;
  m_intercept_input_range_of_min_val = 100;
  m_intercept_input_max_val = 100;
  m_intercept_input_min_val = 0;
  
  m_last_ignore_list_sent = 0.0;

  m_stop_zone_buffer          = 20.0;
  m_cumulative_rad_change     = 0.0; 
  m_region_update_thresh_dist = 10.0;

  
}

//---------------------------------------------------------
// Destructor

HVUProtectNodeCtrl::~HVUProtectNodeCtrl()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool HVUProtectNodeCtrl::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if(key == "NODE_REPORT") {
      bool ok1 = handleNodeReport(msg.GetString());
      if (!ok1)
	reportRunWarning("Unhandled Mail: " + key + ":" + msg.GetString());

    } else if (key == "NAV_X") {
      m_nav_x = msg.GetDouble();
      m_nav_x_vec.push_back( msg.GetDouble() ); 

    } else if (key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
      m_nav_y_vec.push_back( msg.GetDouble() );

    } else if (key == "CONTACTS_LIST") {
        // parse message
      std::vector<std::string> svector = parseString(msg.GetString(), ',');
      for (unsigned int i=0; i<svector.size(); i++) {
	m_contacts.insert(svector[i]);
      }
      m_send_new_ignore_set = true;

    } else if (key == "OPTION") {
      bool ok = handleOptionMsg(msg.GetString());
      if (!ok)
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "INSIDE_ZONE_LIST")  {
      m_inside_zone_list = msg.GetString(); 
      m_send_new_ignore_set = true;

    } else if (key == "OUTSIDE_ZONE_LIST") {
      m_outside_zone_list = msg.GetString();
      m_send_new_ignore_set = true;

    } else if (key == "M300_BATT_VOLTAGE") {
      m_m300_batt_voltage = msg.GetDouble();
      m_got_real_batt_voltage = true;

    } else if (key == "MISSION_TASK"){
      handleMissionTask(msg.GetString());
      
    } else if (key == "ZONE_UP_RAD"){
      double up_val = msg.GetDouble(); 
      m_inside_zone_poly.grow_by_amt(up_val);
      m_outside_zone_poly.grow_by_amt(up_val);
      m_stop_intercept_zone_poly.grow_by_amt(up_val);
      m_cumulative_rad_change += up_val;
      postNewRegionInfo();

    } else if (key == "ZONE_UP_RAD_RESET"){
      m_inside_zone_poly.grow_by_amt(-m_cumulative_rad_change);
      m_outside_zone_poly.grow_by_amt(-m_cumulative_rad_change);
      m_stop_intercept_zone_poly.grow_by_amt(-m_cumulative_rad_change);
      m_cumulative_rad_change = 0.0;
      postNewRegionInfo();
      
    } else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }


  // Init Geodesy 
  GeodesySetup();
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool HVUProtectNodeCtrl::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool HVUProtectNodeCtrl::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  // update m_intruder_names
  updateIntruderList(); 

  // update time-variying options
  updateOptionsActive();


  calculateInputs(MOOSTime());
  publishInputs();

   

  if ( (m_send_new_ignore_set) || ((MOOSTime() -  m_last_ignore_list_sent) > 20.0 ) ){
    publishIgnoreLists(); 
  }


  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool HVUProtectNodeCtrl::OnStartUp()
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
    if(param == "inside_zone") {
        m_inside_zone_poly = string2Poly(value);
	m_inside_zone_poly.set_label("prox_opregion_inside");
	handled = m_inside_zone_poly.is_convex();
	m_inside_zone_centriod = m_inside_zone_poly.get_centroid_pt();
    }
    else if (param == "outside_zone") {
        m_outside_zone_poly = string2Poly(value);
	m_outside_zone_poly.set_label("prox_opregion_inside");
	handled = m_outside_zone_poly.is_convex();
	m_outside_zone_centriod = m_outside_zone_poly.get_centroid_pt();

	m_stop_intercept_zone_poly = m_outside_zone_poly;
	m_stop_intercept_zone_poly.set_label("stop_intercept_zone");
	
    }
    else if (param == "hvu_name") {
      if (value == ""){
	handled = false;
      } else {
	m_hvu_name = value;
	handled = true; 
      }
    }
    else if (param == "stop_zone_buffer") {
      handled = setPosDoubleOnString(m_stop_zone_buffer, value);

    }
    else if (param == "intercept_input_range_of_max_val") {
      handled = setPosDoubleOnString(m_intercept_input_range_of_max_val, value);
    }
    else if (param == "intercept_input_range_of_min_val") {
      handled = setPosDoubleOnString(m_intercept_input_range_of_min_val, value);
    }
    else if (param == "intercept_input_max_val") {
      handled = setPosDoubleOnString(m_intercept_input_max_val, value);
    }
    else if (param == "intercept_input_min_val") {
      handled = setDoubleOnString(m_intercept_input_min_val, value);
    }
    else if (param == "min_batt") {
      handled = setPosDoubleOnString(m_min_batt, value);
    }
    else if (param == "max_batt") {
      handled = setPosDoubleOnString(m_max_batt, value); 
    }
    else if (param == "region_update_thresh_dist") {
      handled = setPosDoubleOnString(m_region_update_thresh_dist, value); 
    }



    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  // grow poly now after collecting all config params
  if (m_outside_zone_poly.is_convex())
    m_stop_intercept_zone_poly.grow_by_amt(m_stop_zone_buffer);

  
  m_last_ignore_list_sent = MOOSTime(); 
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void HVUProtectNodeCtrl::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NODE_REPORT",0);
  Register("NAV_Y", 0);
  Register("CONTACTS_LIST", 0);
  Register("INSIDE_ZONE_LIST", 0);
  Register("OUTSIDE_ZONE_LIST", 0);
  Register("OPTION",0);
  Register("M300_BATT_VOLTAGE",0);
  Register("MISSION_TASK",0);
  Register("ZONE_UP_RAD",0);
  Register("ZONE_UP_RAD_RESET",0);
  
}


//------------------------------------------------------------
// Procedure: buildReport()

bool HVUProtectNodeCtrl::buildReport() 
{
 ACTable actab2(2);
  actab2 << " Odometry | Batt Volts ";
  actab2.addHeaderLines();
  actab2 << doubleToString(m_odometry) << doubleToString(m_current_batt_voltage); 
  m_msgs << actab2.getFormattedString();
  m_msgs << endl;

  if (m_intruder_names.size() > 0){
    m_msgs << "                                                           " << endl;
    m_msgs << "==========================================================" << endl;
    m_msgs << " Intruders: " <<  stringSetToString(m_intruder_names)       << endl;
  }

  if (m_cur_mission_tasks.size() > 0){
    m_msgs << "                                                           " << endl;
    m_msgs << "===========================================================" << endl;
    m_msgs << " Tasks:                                                    "  << endl;
    std::list<MissionTask>::iterator lit;
    for (lit = m_cur_mission_tasks.begin(); lit != m_cur_mission_tasks.end(); lit++){
      m_msgs << " *" << lit->getSpec()  << endl;
    }
    
  }
  
  m_msgs << "                                                           " << endl;
  m_msgs << "===========================================================" << endl;
  m_msgs << "=  Input Values:                                           " << endl;

  std::map<std::string, double>::iterator it;
  for (it = m_opinion_inputs.begin(); it!= m_opinion_inputs.end(); it++) {
    m_msgs << "  " << it->first << ": " <<  doubleToStringX(it->second, 4) << endl; 
  }
  
  return(true);
}





//----------------------------------------------------------
// Procedure: calculateInputs(time)

bool HVUProtectNodeCtrl::calculateInputs(double time)
{

  //////////////////////////////////////////
  // odometry and batter input related
  
  // update odometry
  
  bool same_length = m_nav_x_vec.size() == m_nav_y_vec.size(); 
  if ( (m_nav_x_vec.size() >= 2) && same_length) {
    //
    unsigned int i;
    for (i = 0; i<m_nav_x_vec.size()-1; i++) {
      double dx = m_nav_x_vec[i+1] - m_nav_x_vec[i];
      double dy = m_nav_y_vec[i+1] - m_nav_y_vec[i];
      m_odometry += sqrt( dx * dx + dy * dy);
    }
    
    double last_x = m_nav_x_vec[m_nav_x_vec.size()-1];
    double last_y = m_nav_y_vec[m_nav_y_vec.size()-1];
    m_nav_x_vec.clear();
    m_nav_y_vec.clear();
    m_nav_x_vec.push_back(last_x);
    m_nav_y_vec.push_back(last_y);
    
  }

  
  double max_heron_range = 1 * 60 * 60 * 2;
  if (m_got_real_batt_voltage) {
    
    // Low pass filter. 
    double delta_t = MOOSTime() - m_time_last_got_batt;
    double alpha = alpha = 1.0 - exp( -1.0 * delta_t / 0.5);
    
    m_current_batt_voltage_filtered += alpha * ( m_m300_batt_voltage - m_current_batt_voltage_filtered);
    m_time_last_got_batt = MOOSTime();

    m_current_batt_voltage = m_current_batt_voltage_filtered; 
  } else {
    // fake it
    m_current_batt_voltage = (m_min_batt - m_max_batt) * (m_odometry / max_heron_range) + m_max_batt;
    Notify("M300_SIM_BATT_VOLTAGE",m_current_batt_voltage);
  }

  // battery_exhausted = 0 fresh battery
  //                   = 1 dead  battery
  
  double battery_exhausted = (m_max_batt - m_current_batt_voltage)/ (m_max_batt - m_min_batt);

  
  // when battery is exhausted 
  //m_opinion_inputs["OUTSIDE_ZONE_INPUT"] = 50.0 * battery_exhausted * battery_exhausted + 50.0;
  m_opinion_inputs["INSIDE_ZONE_INPUT"]  = 50.0 * battery_exhausted + 50.0;
  m_opinion_inputs["OUTSIDE_ZONE_INPUT"] = -50.0 * battery_exhausted + 50.0;


  // Notify shoreside for metric
  std::string metric_spec = "name=" + m_host_community + ",odom=" + doubleToString(m_odometry,2) + ",batt=" + doubleToString(m_current_batt_voltage);
  Notify("NODE_STATUS_MSG", metric_spec); 
  
  //////////////////////////////////////////
  // intercept related
  
  // Are there any intruders?
  if (m_intruder_names.size() == 0){
    // clear the input map
    if (m_opinion_inputs.count("INTERCEPT_INPUT") > 0) {
      m_opinion_inputs.erase("INTERCEPT_INPUT");
    }
    // clear the name
    m_closest_intruder_name = "";
    
    return(true);
  }

  // update the closest intruder info
  double closest_intruder_dist = updateClosestIntruder();
  Notify("CLOSEST_INTRUDER_DIST", closest_intruder_dist); 

  // calcuate input
  double intercept_input = 0.0;

  if (closest_intruder_dist > m_intercept_input_range_of_min_val){
    // we are very far away
    intercept_input = m_intercept_input_min_val; 

  } else if (closest_intruder_dist < m_intercept_input_range_of_max_val) {
    // we are very close
    intercept_input = m_intercept_input_max_val;
    
  } else {
    double span = m_intercept_input_range_of_min_val - m_intercept_input_range_of_max_val;
    if (span < 0)
      span *= -1.0;

    double delta = closest_intruder_dist - m_intercept_input_range_of_max_val; // will always be non negative
    double proportion = delta / span;

    // proportion from [0 1] where zero is really close

    intercept_input = proportion * (m_intercept_input_min_val - m_intercept_input_max_val) + m_intercept_input_max_val;
  }

  // add it
  m_opinion_inputs["INTERCEPT_INPUT"] = intercept_input;
 
  
  return(true); 
}



//----------------------------------------------------------
// Procedure: calculateInputs(time)

bool HVUProtectNodeCtrl::publishInputs()
{

  std::map<std::string, double>::iterator it;
  for (it = m_opinion_inputs.begin(); it!= m_opinion_inputs.end(); it++) {
    Notify(it->first, it->second); 
  }
  
  return(true); 
}



//---------------------------------------------------------
// Procedure: GeodesySetup()
//   Purpose: Initialize geodesy object with lat/lon origin.
//            Used for LatLon2LocalUTM conversion.

bool HVUProtectNodeCtrl::GeodesySetup()
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


//---------------------------------------------------------------
// publishIgnoreLists()
//         Any and all
bool HVUProtectNodeCtrl::publishIgnoreLists()
{
  // first set the active list
  if (m_cur_option == "inside_zone"){
    buildVoronoiActiveList(m_inside_zone_list); 
    
  } else if (m_cur_option == "outside_zone") {
    buildVoronoiActiveList(m_outside_zone_list); 

  } else {
    return(false); 
  }

  std::string val = "";
  std::set<std::string>::iterator it;
  for (it = m_contacts.begin(); it != m_contacts.end(); it++){
    if (m_voronoi_active.count(*it)==0) {
      val += *it + ",";
    }
  }

  // remove the last comma if needed
  if (val.length()>0)
    val = val.substr(0, val.size()-1);

  Notify("PROX_SET_IGNORE_LIST", val);
  m_send_new_ignore_set = false;

  m_last_ignore_list_sent = MOOSTime(); 
  
  return(true);
}


//----------------------------------------------------
// handleVoronoiActiveList(std::string val)
//
bool HVUProtectNodeCtrl::buildVoronoiActiveList(std::string msg)
{
  // Make a new set to erase the old set.
  std::set<std::string> new_voronoi_active_set;
  // parse message
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i=0; i<svector.size(); i++) {
    new_voronoi_active_set.insert(svector[i]);
  }

  if (true) {
    new_voronoi_active_set.insert(m_hvu_name); 
  }
  
  m_voronoi_active = new_voronoi_active_set;
      
  return(true); 
}

//-----------------------------------------------------
// Handle Option message
//
bool HVUProtectNodeCtrl::handleOptionMsg(std::string msg)
{
  // bail right away if this msg is not a change
  if (msg == m_cur_option)
    return(true);

  m_cur_option = tolower(msg);
  m_send_new_ignore_set = true; 

  bool ok = true; 
  if (m_cur_option != "intercept"){
    ok = postNewRegionInfo();
  }
 
  return(ok); 
}


//-----------------------------------------------------
// Handle Node Report
//
bool HVUProtectNodeCtrl::handleNodeReport(std::string msg)
{

 
  // process incoming node record 
  NodeRecord newNodeRecord;
  newNodeRecord = string2NodeRecord(msg, true);
  Notify("TMP_DEBUG", newNodeRecord.getSpec());
  if (!newNodeRecord.valid())
    return(false);

  // First save it
  m_node_rec_map[newNodeRecord.getName()] = newNodeRecord;

  // now check if it s the HVU and update the region accordingly
  if (newNodeRecord.getName() != m_hvu_name)
    return(true); 
  
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
  double dist_inside, dist_outside;
  dist_inside = distPointToPoint( m_inside_zone_centriod.get_vx(),
				  m_inside_zone_centriod.get_vy(),
				  node_x, node_y);
  dist_outside = distPointToPoint( m_outside_zone_centriod.get_vx(),
				   m_outside_zone_centriod.get_vy(),
				   node_x, node_y);

  
  bool ok2 = true;
  bool cond1 = (dist_inside > m_region_update_thresh_dist);
  bool cond2 = (dist_outside > m_region_update_thresh_dist);
  if ( cond1 || cond2 ) {
    m_inside_zone_poly.new_centroid(node_x, node_y);
    m_outside_zone_poly.new_centroid(node_x, node_y);

    m_stop_intercept_zone_poly = m_outside_zone_poly;
    m_stop_intercept_zone_poly.grow_by_amt(m_stop_zone_buffer);

    // also update the centriod points
    m_inside_zone_centriod = m_inside_zone_poly.get_centroid_pt();
    m_outside_zone_centriod = m_outside_zone_poly.get_centroid_pt();
    
    ok2 = postNewRegionInfo();
  }

  return(ok2); 

}


//----------------------------------------------
// Procedure:  handleMissionTask()
//             Also check if the task is to stop
//             intercepting, and if so, it removes any
//             tasks in m_curr_tasks

void HVUProtectNodeCtrl::handleMissionTask(std::string msg){
  
  MissionTask newMissionTask = string2MissionTask(msg);

  // Is this a new intercept message?
  std::string task_type = newMissionTask.getTaskDetails();

  double x, y;
  std::string name;
  bool stop;
  parseMissionTask(task_type, stop, name, x, y);

  if (name =="to-be-set"){
    // we are not concerned, this must not be an intercept task
    return; 
  }

  if (!stop) {
    // this is a new intercept message; add it.
    m_cur_mission_tasks.push_back(newMissionTask);
  } else {
    // This is a stop intercept task. We need to remove all other tasks
    // with this name.
    double this_task_x, this_task_y;
    std::string this_task_name;
    bool this_task_stop;
    
    std::list<MissionTask>::iterator it1;
    for (it1 = m_cur_mission_tasks.begin(); it1 != m_cur_mission_tasks.end(); ){
      std::string this_task_type;
      this_task_type = it1->getTaskDetails();
      parseMissionTask(this_task_type, this_task_stop, this_task_name, this_task_x, this_task_y);
      
      if (this_task_name == name){
	// we've found a task that we need to stop
	 it1 = m_cur_mission_tasks.erase(it1); 
      } else {
	//increment to the next
	++it1;
      } 
    }
    
  }
  
  return;
}


//----------------------------------------------------
// Post new region info
//
bool HVUProtectNodeCtrl::postNewRegionInfo()
{

  m_stop_intercept_zone_poly.set_color("vertex","orange");
  m_stop_intercept_zone_poly.set_color("edge","yellow");
  m_stop_intercept_zone_poly.set_label("stop_intercept_zone");
  
  if (m_cur_option == "inside_zone"){
    if (m_inside_zone_poly.is_convex()){
      Notify("PROX_UP_REGION", m_inside_zone_poly.get_spec());
      Notify("VIEW_POLYGON", m_stop_intercept_zone_poly.get_spec());
    }
    
  } else if (m_cur_option == "outside_zone") {
    if (m_outside_zone_poly.is_convex()){
      Notify("PROX_UP_REGION", m_outside_zone_poly.get_spec());
      Notify("VIEW_POLYGON", m_stop_intercept_zone_poly.get_spec());
    }

  } 

  return(true); 

}






//------------------------------------------------
// procedure:  updateIntruderList()
//             Updates m_intruder_names 

void HVUProtectNodeCtrl::updateIntruderList()
{

  // first check the easy case where there are no
  // active mission tasks. 
  if (m_cur_mission_tasks.size() == 0){
    m_intruder_names.clear();
    return;
  }

  // now build a new set of names.
  std::set<std::string> new_intruder_names;

  std::list<MissionTask>::iterator it1;
  double x, y;
  std::string name;
  bool stop;


  for (it1 = m_cur_mission_tasks.begin(); it1 != m_cur_mission_tasks.end(); it1++){
    std::string this_task_type;
    this_task_type = it1->getTaskDetails();
    parseMissionTask(this_task_type, stop, name, x, y);

    //sanity check here, there should be no stop mission tasks
    if ((!stop) && (name != "to-be-set")) {
      new_intruder_names.insert(name);
    }
  }

  m_intruder_names = new_intruder_names; 
  
  return;
}


//------------------------------------------------
// procedure:  updateOptionsActive()
//             sends out the approprate messages
//             to enable the opinion dynamics
//             Also, if no nodeReport is available
//             pass along the estimated x and y position
void HVUProtectNodeCtrl::updateOptionsActive()
{

  if (m_intruder_names.size() == 0){
    Notify("ACTIVE_INTERCEPT","false");
    Notify("INTRUDER_LIST", "");
    return;
  }

  Notify("ACTIVE_INTERCEPT","true");
  
  // send out the new intruder list
  std::string intruder_list = stringSetToString(m_intruder_names);
  Notify("INTRUDER_LIST", intruder_list);

  // Send out a estimated x and y position for the
  // allocation
  std::list<MissionTask>::iterator it1;
  double x, y;
  std::string name;
  bool stop;


  for (it1 = m_cur_mission_tasks.begin(); it1 != m_cur_mission_tasks.end(); it1++){
    std::string this_task_type;
    this_task_type = it1->getTaskDetails();
    parseMissionTask(this_task_type, stop, name, x, y);

    //sanity check here, there should be no stop mission tasks
    if ((stop) || (name == "to-be-set"))
      continue;

    // check if we have a node report for this vehicle
    //if ( m_node_rec_map.count(name) == 0){
      // We do not have a node report for this vehicle,
      // perhaps it is out of range.  Send the info we
      // got from the task message. 
      
      // build the msg
      std::string short_task_msg = "name=" + name;
      short_task_msg += ",x=" + to_string(x);
      short_task_msg += ",y=" + to_string(y);
      Notify("INTERCEPT_ESTIMATE", short_task_msg);
      //}
    
  }
  
  return;
}



//---------------------------------------------
// Procedure:  parseMissionTask
//             intercept=ike,x=10,y=20
//             or stop_intercept=ike

void HVUProtectNodeCtrl::parseMissionTask(std::string spec, bool &stop, std::string &name, double &x, double &y)
{
  double new_x = 0.0;
  double new_y = 0.0; 
  bool new_stop = false;
  std::string new_name = "to-be-set";

  vector<string> svector = parseString(spec, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    if((param == "intercept") && (value != ""))
      new_name = value;
    else if((param == "stop_intercept") && (value != "")){
      new_name = value;
      new_stop = true;
    }
    else if((param == "x") && isNumber(value))
      setDoubleOnString(new_x, value);
    else if((param == "y") && isNumber(value))
      setDoubleOnString(new_y, value);
  }

  //update by reference
  stop = new_stop;
  name = new_name;
  x = new_x;
  y = new_y; 
  
  return;
}




//-----------------------------------------
// Procedure: updateClosetsIntruder
//            Updates m_closest_intruder_name
//            Returns the distance to the closest
//            intruder


double HVUProtectNodeCtrl::updateClosestIntruder()
{
  // update the closest intruder
  std::list<MissionTask>::iterator lit;

  double intrd_x, intrd_y;
  std::string intrd_name;
  bool stop;

  double closest_intruder_dist = std::numeric_limits<double>::max();
  
  for (lit = m_cur_mission_tasks.begin(); lit != m_cur_mission_tasks.end(); lit++){

    std::string task_type = lit->getTaskDetails();
    parseMissionTask(task_type, stop, intrd_name, intrd_x, intrd_y);

    // check if we have a node report for this contact.

    if (m_node_rec_map.count(intrd_name) > 0){
      // use the x and y in this node report instead of the data
      // in the mission description
      intrd_x = m_node_rec_map[intrd_name].getX();
      intrd_y = m_node_rec_map[intrd_name].getY();
     }

    double dist = distPointToPoint(intrd_x, intrd_y, m_nav_x, m_nav_y);

    if (dist < closest_intruder_dist){
      m_closest_intruder_name = intrd_name;
      closest_intruder_dist = dist; 
    }
  }
  
  return(closest_intruder_dist); 
}
