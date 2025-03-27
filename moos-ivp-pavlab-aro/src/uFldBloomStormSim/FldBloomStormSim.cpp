/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: FldBloomStormSim.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "FldBloomStormSim.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

FldBloomStormSim::FldBloomStormSim()
{
   m_bloom_rad_max = 200;
   m_bloom_rad_min = 100;
   m_bloom_duration_max = 360;
   m_bloom_duration_min = 300;
   m_bloom_max_speed = 0.5;
   m_last_iteration_time = MOOSTime(); 
   
   m_sensor_decay_parameter = 1.0; 
   m_probability_of_new_bloom = 0.1;

   m_allow_storms = true;
   m_storm_speed = 1;
   m_storm_heading = 0;
   m_storm_max_angle = 20;
   m_time_between_storms = 1000;
   m_storm_x = 0.0;
   m_storm_y = 0.0;
   m_storm_last_update_time = MOOSTime();
   m_storm_active = false;
   m_storm_radius = 100;
   m_storm_count = 0; 

   m_last_storm_time = MOOSTime();
   m_last_bloom_start_time = MOOSTime();
   m_bloom_count = 0;
   
   m_sampled_blooms_completed = 0; 
   m_undetected_and_unsampled_blooms_completed = 0;
   m_detected_blooms_completed = 0;

   m_only_post_in_region_one = false;
   
   m_eval_started = false;
   m_last_eval_time = MOOSTime();
   m_mixed_strat_run_avg =  0.0;
   m_mixed_strat_total_running_time = 0.0;
   m_deployed = false;
   

}

//---------------------------------------------------------
// Destructor

FldBloomStormSim::~FldBloomStormSim()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool FldBloomStormSim::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

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
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
    bool handled = false;
    if((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL")){
      handled = handleMailNodeReport(sval);
    }
    else if (key == "SAMPLE_FINISHED_LOG") {
      handled = handleSampleFinishedLog(sval); 
    }
    else if (key == "DEPLOY_ALL"){
      handled = setBooleanOnString(m_deployed, sval);
    }
    else if (m_vehicles_registered_for_option.count(key) > 0){
      // get the vehicle name by removing the "OPTION_"
      std::string vname;
      if (key.size() > 7){
        vname = tolower(key.erase(0,7));
	m_vechicle_opt_status[vname] = toupper(sval);
	handled = true;
      }
    }
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
      
    if (!handled)
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool FldBloomStormSim::OnConnectToServer()
{
   registerVariables();
   Notify("UNSAMP_UNDETECT_BLOOMS_COMPLETED", uintToString(0));  
   Notify("SAMPLED_BLOOMS_COMPLETED", uintToString(0));
   Notify("DETECTED_BLOOMS_COMPLETED", uintToString(0));
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool FldBloomStormSim::Iterate()
{

  AppCastingMOOSApp::Iterate();

  manageBlooms();
  notifyVehiclesOfBloom();

  if (m_allow_storms){
    manageStorm();
    notifyVehiclesOfStorm();
  }
  
  postEval(); 

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool FldBloomStormSim::OnStartUp()
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
    if(param == "region1") {
      handled = handleConfigRegion(value, 1);
    }
    else if(param == "region2") {
     handled = handleConfigRegion(value, 2);
    }
    else if(param == "bloom_rad_max") {
      handled = setPosDoubleOnString(m_bloom_rad_max, value);
    }
    else if(param == "bloom_rad_min") {
      handled = setPosDoubleOnString(m_bloom_rad_min, value);
    }
    else if(param == "bloom_duration_min") {
      handled = setPosDoubleOnString(m_bloom_duration_min, value);
    }
    else if(param == "bloom_duration_max") {
      handled = setPosDoubleOnString(m_bloom_duration_max, value);
    }
    else if(param == "probability_of_new_bloom") {
      handled = setPosDoubleOnString(m_probability_of_new_bloom, value);
    }
    else if(param == "bloom_max_speed") {
      handled = setPosDoubleOnString(m_bloom_max_speed, value);
    }
    else if(param == "time_between_bloom_attempts") {
      handled = setPosDoubleOnString(m_time_between_bloom_attempts, value);
    }
    else if(param == "allow_storms") {
      handled = setBooleanOnString(m_allow_storms, value);
    }
    else if(param == "storm_speed") {
      handled = setPosDoubleOnString(m_storm_speed, value);
    }
    else if(param == "storm_max_angle") {
      handled = setPosDoubleOnString(m_storm_max_angle, value);
    }
    else if(param == "storm_radius") {
      handled = setPosDoubleOnString(m_storm_radius, value);
    }
    else if(param == "time_between_storms") {
      handled = setPosDoubleOnString(m_time_between_storms, value);
    }
    else if(param == "sensor_decay_parameter") {
      handled = setPosDoubleOnString( m_sensor_decay_parameter, value);
    }
    else if(param == "only_post_in_region_one") {
      handled = setBooleanOnString(m_only_post_in_region_one, value); 
    }
    else if(param == "storm_region") {
        m_storm_region = string2Poly(value);
	m_storm_region.set_label("storm_region");
	if (m_storm_region.is_convex()) {
	  Notify("VIEW_POLYGON",m_storm_region.get_spec());
	  handled = true; 
	} else {
	  handled = false; 
	}
    }
    

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  unsigned long int tseed = time (NULL);
  unsigned long int pseed = getpid()+1;
  unsigned int rseed = (tseed*pseed) % 50000;
  srand(rseed);

  // shrink polygons by the max radius
  m_region_1.grow_by_amt(-m_bloom_rad_max);
  m_region_2.grow_by_amt(-m_bloom_rad_max);
  m_region_1.set_label( m_region_1.get_label() + "_inner");
  m_region_2.set_label( m_region_2.get_label() + "_inner"); 
  //Notify("VIEW_POLYGON", m_region_1.get_spec());
  //if (!m_only_post_in_region_one)
  //  Notify("VIEW_POLYGON", m_region_2.get_spec());

  m_storm_circle.setRad(m_storm_radius);
  m_storm_circle.set_active(m_storm_active);
  m_storm_circle.set_label("Storm_"+ uintToString(m_storm_count));
  m_storm_circle.set_color("edge", "white");
  m_storm_circle.set_color("fill", "black");
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void FldBloomStormSim::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
  Register("SAMPLE_FINISHED_LOG", 0);
  Register("DEPLOY_ALL", 0);
}

//--------------------------------------------------------- 
// Procedure: handleConfigRegion                   

bool FldBloomStormSim::handleConfigRegion(std::string opstr, unsigned int number)
{
  XYPolygon new_region = string2Poly(opstr);
  string number_str = uintToString(number);
  
  new_region.set_label("survey_region_" + number_str);

  if(!new_region.is_convex())
    return(false);

  if (number == 1){
    m_region_1 = new_region;
    // Notify("VIEW_POLYGON", m_region_1.get_spec());
  } else if (number == 2) {
    m_region_2 = new_region;
    //if (!m_only_post_in_region_one)
    // Notify("VIEW_POLYGON", m_region_2.get_spec());
  } else {
    return(false);
  }
  return(true);
}

//-----------------------------------------------------------
// Procedure: manageBlooms()
bool FldBloomStormSim::manageBlooms()
{


  // Check if any bloom has expired
  checkActiveBlooms(); 

  // Generate a new bloom if necessary
  generateNewBloom();

  // Move the blooms
  moveBlooms(); 
  
  
  return(true); 
}


//----------------------------------------------------------
// Procedure:
bool FldBloomStormSim::generateNewBloom()
{
  // Time for a new bloom?
  bool new_bloom = false;
  if ( ( MOOSTime() - m_last_bloom_start_time) > m_time_between_bloom_attempts) {
    
    // roll the dice
    int thresh = static_cast<int>(100.0 * m_probability_of_new_bloom); 
    int roll = rand() % 100;
    if ( roll < thresh ) {
      new_bloom = true;
    }
  }

  // if no new bloom, then exit.
  if (!new_bloom){
    return(true); 
  }
  
  // if a new bloom, then find a new random location for the bloom
  double ptx = 0;
  double pty = 0;
  double new_pulse_rad = 0;
  
  bool good_point_found = false;
  int max_count = 10;
  int count = 0;
  while (!good_point_found) {
    
    // Find a possible new location and pulse range
    bool ok = false;

    if (((rand() % 100) > 50) || (m_only_post_in_region_one)) {
      XYPolygon shrunk_region = m_region_1;
      shrunk_region.grow_by_amt(-1.0 * m_bloom_rad_max);
      ok = randPointInPoly(shrunk_region, ptx, pty);
    } else {
      ok = randPointInPoly(m_region_2, ptx, pty);
    }
    
    if(!ok) {
      reportRunWarning("Problem generating new random point for bloom");
    }
    
    double rand_rad_scale = static_cast<double>(rand()) / static_cast<double>(RAND_MAX); 
    new_pulse_rad = m_bloom_rad_min + rand_rad_scale * (m_bloom_rad_max - m_bloom_rad_min);
    
    good_point_found = true; 
    // Check if it will intersect with an existing bloom
    std::vector<XYCircle>::iterator it;
    for (it = m_active_blooms.begin(); it != m_active_blooms.end(); it++){
      XYCircle circle= *it;
      
      double dx = circle.getX() - ptx;
      double dy = circle.getY() - pty;
      double dist = sqrt(dx*dx + dy*dy);
      
      if (dist < ( circle.getRad() + new_pulse_rad)) {
	// This choice wont work.
	good_point_found = false;
	break; 
      }
    }

    count +=1;
    
    if (count > max_count) {
      // we can't find a location
      return(true);
    }
  } // end of while loop

  
  // Add this new good bloom to the list and publish.
  double rand_dur_scale = static_cast<double>(rand()) / static_cast<double>(RAND_MAX); 
  double new_pulse_dur = m_bloom_duration_min + rand_dur_scale * (m_bloom_duration_max - m_bloom_duration_min);

  m_bloom_count +=1;
  
  XYCircle new_bloom_marker;
  new_bloom_marker.setX(ptx);
  new_bloom_marker.setY(pty);
  new_bloom_marker.setRad(new_pulse_rad);
  new_bloom_marker.set_duration(new_pulse_dur);
  new_bloom_marker.set_label("Bloom_"+ uintToString(m_bloom_count));
  new_bloom_marker.set_color("edge", "white");
  new_bloom_marker.set_color("fill", "red");
  new_bloom_marker.set_time(MOOSTime());
  
  Notify("VIEW_CIRCLE", new_bloom_marker.get_spec());

  m_active_blooms.push_back(new_bloom_marker);

  // also generate a random velocity and save it
  // generate random vals
  double rand_n1 = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
  double rand_n2 = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  double rand_vx = (rand_n1 - 0.5) * 2.0 * m_bloom_max_speed;
  double rand_vy = (rand_n2 - 0.5) * 2.0 * m_bloom_max_speed; 
  m_bloom_vx.push_back(rand_vx);
  m_bloom_vy.push_back(rand_vy); 
  
  m_last_bloom_start_time = MOOSTime();

  return(true);
}

//----------------------------------------------------------
// Procedure:
bool FldBloomStormSim::moveBlooms()
{

  double dt = MOOSTime() - m_last_iteration_time;

  if (m_active_blooms.size() != m_bloom_vx.size())
    return(true);

  if (m_bloom_vx.size() != m_bloom_vy.size())
    return(true); 
  
  for (int i = 0; i < m_active_blooms.size(); i++){

    double vx = m_bloom_vx[i];
    double vy = m_bloom_vy[i];

    m_active_blooms[i].setX( vx* dt + m_active_blooms[i].getX());
    m_active_blooms[i].setY( vy* dt + m_active_blooms[i].getY()); 

    Notify("VIEW_CIRCLE", m_active_blooms[i].get_spec());
  }

  m_last_iteration_time = MOOSTime(); 
  return(true); 
}


//------------------------------------------------------------
// Procedure: checkActiveBlooms()
bool FldBloomStormSim::checkActiveBlooms()
{
  
  std::vector<XYCircle>::iterator it;
  for (it = m_active_blooms.begin(); it != m_active_blooms.end();){
    
    if ((MOOSTime() - it->get_time()) > it->get_duration()){
      
      // This bloom has expired
      bool was_detected = (m_blooms_detected_labels.count(it->get_label()) > 0);
      bool was_sampled = (m_blooms_sampled_labels.count(it->get_label()) > 0);

      // assume if was sampled it was detected.
      if (was_sampled)
	m_sampled_blooms_completed += 1;
      else if (was_detected)
	m_detected_blooms_completed += 1;
      else
	m_undetected_and_unsampled_blooms_completed += 1;

      Notify("UNSAMP_UNDETECT_BLOOMS_COMPLETED", uintToString(m_undetected_and_unsampled_blooms_completed));  
      Notify("SAMPLED_BLOOMS_COMPLETED", uintToString(m_sampled_blooms_completed));
      Notify("DETECTED_BLOOMS_COMPLETED", uintToString(m_detected_blooms_completed));
      it->set_active(false); 
      Notify("VIEW_CIRCLE", it->get_spec()); 
 
      // what position is this?
      int index = std::distance(m_active_blooms.begin(), it);
      // clear the velocity positions, need to make this a class
      m_bloom_vx.erase( m_bloom_vx.begin() + index);
      m_bloom_vy.erase( m_bloom_vy.begin() + index);
      
      it = m_active_blooms.erase(it);
    } else { 
      it++;
    }
  }
  return(true); 
}



//-----------------------------------------------------------
// Procedure: notifyVehiclesOfBloom()
bool FldBloomStormSim::notifyVehiclesOfBloom()
{

  m_vehicles_in_bloom.clear();
  m_bloom_sensor_reading.clear();

  // Assume cost to assemble the polygon makes the order of this nested iteration
  // optimal. 
  std::vector<XYCircle>::iterator it2;
  double bloom_range, bloom_x, bloom_y;
  double dist; 
  
  for (it2 = m_active_blooms.begin(); it2 != m_active_blooms.end(); it2++) {

    // check if this record is within a bloom circle
    bloom_range = it2->getRad();  // getRange(*it2, MOOSTime());

    bloom_x = it2->getX();
    bloom_y = it2->getY();

    // is this already sampled?

    
    std::map<std::string, NodeRecord>::iterator it;
    for ( it = m_map_node_records.begin(); it != m_map_node_records.end(); it++) {
      
      // to save time
      if (m_vehicles_in_bloom.count(it->first) >0)
	continue;
      double vx = it->second.getX();
      double vy = it->second.getY();
      
      dist = distPointToPoint(bloom_x, bloom_y, vx, vy);
      
      if (dist < bloom_range) {
	m_vehicles_in_bloom.insert(it->first);
	// add this bloom label to the set of blooms detected
	m_blooms_detected_labels.insert(it2->get_label());
	// Also mark as notified

	// is this sampled?
	if ( m_blooms_sampled_labels.count(it2->get_label()) == 0){
	  it2->set_color("edge", "pink");
	  it2->set_color("fill", "pink");
	  Notify("VIEW_CIRCLE", it2->get_spec());
	}
	
      } else
	Notify("RANGE_TO_BLOOM", dist);

      Notify("BLOOM_RANGE", bloom_range); 
      
      // also determine the sensor input at this range
      double sensor_reading = 0.0;
      
      if (dist <= bloom_range){
	sensor_reading = 1.0;
      } else {
	sensor_reading = 1.0 * (1.0 / (m_sensor_decay_parameter *(dist - bloom_range + 1.0)));
      }
      // save the sensor reading if the largest so far
      if (m_bloom_sensor_reading.count(it->first) > 0){
	// is this larger?
	if (m_bloom_sensor_reading[it->first] < sensor_reading)
	  m_bloom_sensor_reading[it->first] = sensor_reading;
      } else {
	m_bloom_sensor_reading[it->first] = sensor_reading;
      }
      
    } // for all node records
  } // for all blooms
  
  // Notify all vehicles in the set.
  std::map<std::string, NodeRecord>::iterator it3;
  for ( it3 = m_map_node_records.begin(); it3 != m_map_node_records.end(); it3++) {
    std::string vname = it3->first; 
    if (m_vehicles_in_bloom.count(vname) > 0) {
      Notify("BLOOM_DETECTED_" + toupper(vname), "true"); 
    } else {
      Notify("BLOOM_DETECTED_" + toupper(vname), "false"); 
    }

    if (m_bloom_sensor_reading.count(vname) > 0){
      Notify("SENSOR_MEAS_" + toupper(vname), m_bloom_sensor_reading[vname]);
    }
  }
  
  return(true); 
}




//-----------------------------------------------------------
// Procedure: manageStorm()
bool FldBloomStormSim::manageStorm()
{

  // Check if a storm is active, and adjust the circle as needed
  checkActiveStorm(); 

  // Time for a new storm?
  bool cond1 = (MOOSTime() - m_last_storm_time) < m_time_between_storms; 
  if (cond1 || m_storm_active) {
    return(true);
  }

  // Otherwise make a new storm
  bool storm_in_region_1 = false; 
  if ((rand() % 100) > 50) {
    storm_in_region_1 = true;
  }

  double sx = 0.0;
  double sy = 0.0;
  bool ok = false; 
  if (storm_in_region_1) {
     ok = randPointInPoly(m_region_1, sx, sy);
  } else {
     ok = randPointInPoly(m_region_2, sx, sy);
  }
  
  if(!ok) {
    reportRunWarning("Problem generating new random point for storm");
  }
  // select a heading
  double min_ang = -m_storm_max_angle; 
  double max_ang =  m_storm_max_angle;
  
  double rand_angle_delta = static_cast<double>(rand()) / static_cast<double>(RAND_MAX); 
  m_storm_heading = min_ang + rand_angle_delta  * (max_ang - min_ang);

  m_storm_heading = 360.0-26.0; 
  // flip north to south
  if ((rand() % 100) > 50) {
    m_storm_heading += 180; 
  }

  // find the start point of the storm on the edge of the storm region
  double min_cpa = polyRayCPA(sx, sy, m_storm_heading, m_storm_region, m_storm_x, m_storm_y);
  m_storm_active = true;
  m_storm_count +=1;
  
  m_storm_circle.set_active(m_storm_active); 
  m_storm_circle.set_label("Storm_"+ uintToString(m_storm_count));

  return(true);
}



//-----------------------------------------------------------
// Procedure: checkActiveStorm()
bool FldBloomStormSim::checkActiveStorm()
{

  if (!m_storm_active){
    m_storm_last_update_time = MOOSTime(); 
    return(true);
  }

  // propagate the storm center
  double dt = MOOSTime() - m_storm_last_update_time;
  double ds = dt * m_storm_speed;
  
  m_storm_x = m_storm_x - ds * sin( m_storm_heading / 180 * PI);
  m_storm_y = m_storm_y - ds * cos( m_storm_heading / 180 * PI); 

  m_storm_circle.setX(m_storm_x);
  m_storm_circle.setY(m_storm_y);
  
  //finally, check if this propagation is outide of the storm region
  if(!m_storm_region.contains(m_storm_x, m_storm_y)){
    m_storm_active = false;
    m_storm_circle.set_active(m_storm_active);
    m_last_storm_time = MOOSTime(); 
  } 

  Notify("VIEW_CIRCLE", m_storm_circle.get_spec());
  m_storm_last_update_time = MOOSTime();
  
  return(true);
  
}


//----------------------------------------------------------
// Procedure: notifyVehiclesOfStorm()
//            update: also sends a sensor message
bool FldBloomStormSim::notifyVehiclesOfStorm() {

  m_vehicles_in_storm.clear();

  if (m_storm_active) {
    std::map<std::string, NodeRecord>::iterator it;
    for ( it = m_map_node_records.begin(); it != m_map_node_records.end(); it++) {
      
      double vx = it->second.getX();
      double vy = it->second.getY();
      
      if (m_storm_circle.containsPoint(vx, vy)) {
	m_vehicles_in_storm.insert(it->first); 
      }

    }
  }

  // Notify all vehicles in the set.
  std::map<std::string, NodeRecord>::iterator it3;
  for ( it3 = m_map_node_records.begin(); it3 != m_map_node_records.end(); it3++) {
    std::string vname = it3->first; 
    if (m_vehicles_in_storm.count(vname) > 0) {
      Notify("STORM_DETECTED_" + toupper(vname), "true"); 
    } else {
      Notify("STORM_DETECTED_" + toupper(vname), "false"); 
    }
  }
  return(true); 
}
//---------------------------------------------------------
// Procedure: handleMailNodeReport()
//   Example: NAME=alpha,TYPE=KAYAK,UTC_TIME=1267294386.51,
//            X=29.66,Y=-23.49,LAT=43.825089, LON=-70.330030, 
//            SPD=2.00, HDG=119.06,YAW=119.05677,DEPTH=0.00,     
//            LENGTH=4.0,MODE=ENGAGED

bool FldBloomStormSim::handleMailNodeReport(const string& node_report_str)
{
  NodeRecord new_record = string2NodeRecord(node_report_str);

  if(!new_record.valid()) {
    reportRunWarning("ERROR: Unhandled node record");
    return(false);
  }

  // In case there is an outstanding RunWarning indicating the lack
  // of a node report for a given vehicle, retract it here. This is 
  // mostly a startup timing issue. Sometimes a sensor request is 
  // received before a node report. Only a problem if the node report
  // never comes. Once we get one, it's no longer a problem.
  string vname = new_record.getName();
  retractRunWarning("No NODE_REPORT received for " + vname);
  
  m_map_node_records[vname] = new_record;

  // Register for incoming option variables if needed
  if (m_vehicles_registered_for_option.count(vname) == 0){
    std::string message_spec = "OPTION_" + tolower(vname);
    Register(message_spec,0);
    m_vehicles_registered_for_option.insert(message_spec); 
  }

  return(true);
}


//-------------------------------------------------------------
// Procedure:  handleSampleFinishedLog()
//             Determines if a vehicle has completed a sample in
//             one of the remaining blooms

bool FldBloomStormSim::handleSampleFinishedLog(std::string vname)
{

  
  std::map<std::string, NodeRecord>::iterator it;
  double vx, vy; 
  it = m_map_node_records.find(vname);

  if (it == m_map_node_records.end())
      return(true);

  vx = it->second.getX();
  vy = it->second.getY();
  
  int count = 0;
  int marked_bloom = 0;
  std::string circle_spec = ""; 

  // now check the active_blooms
  std::string bloom_label;
  double bloom_range;
  double bloom_x;
  double bloom_y;
  double dist;

  for (int i = 0; i<m_active_blooms.size(); i++){
    bloom_label = m_active_blooms[i].get_label(); 
    count += 1;

    if ( m_blooms_sampled_labels.count(bloom_label) > 0) {
      // this is already sampled
      continue; 
    }

    bloom_range = getRange(m_active_blooms[i], MOOSTime());
    bloom_x = m_active_blooms[i].getX();
    bloom_y = m_active_blooms[i].getY();

    dist = distPointToPoint(bloom_x, bloom_y, vx, vy);

    if (dist < bloom_range) {
      // This bloom was sampled, we are done
      m_blooms_sampled_labels.insert(bloom_label);
      
      m_active_blooms[i].set_color("edge", "limegreen");
      m_active_blooms[i].set_color("fill", "limegreen");
      Notify("VIEW_CIRCLE", m_active_blooms[i].get_spec());
      marked_bloom = i;
      circle_spec = m_active_blooms[i].get_spec(); 
      
      
      break; 
    }
    
  }

  return(true);
}

//-------------------------------------------------------------
// Procedure:  handleSampleFinishedLog()
//             Determines if a vehicle has completed a sample in
//             one of the remaining blooms

bool FldBloomStormSim::handleSampleFinishedLogOld(const std::string vname)
{

  std::map<std::string, NodeRecord>::iterator it;
  double vx, vy; 
  it = m_map_node_records.find(vname);

  int count = 0;
  if (it != m_map_node_records.end()) {
    // Check if this vehicle is within any bloom at the moment
    vx = it->second.getX();
    vy = it->second.getY();

    std::vector<XYCircle>::iterator it2;
    std::string bloom_label;
    double bloom_range;
    double bloom_x;
    double bloom_y;
    double dist;

    for (it2 = m_active_blooms.begin(); it2 != m_active_blooms.end(); it2++){
      bloom_label = it2->get_label();
      count += 1;
	
      if ( m_blooms_sampled_labels.count(bloom_label) > 0) {
	// this is already sampled
	continue; 
      }

      
      // check if this record is within a bloom circle
      bloom_range = getRange(*it2, MOOSTime());

      bloom_x = it2->getX();
      bloom_y = it2->getY();

      dist = distPointToPoint(bloom_x, bloom_y, vx, vy);

      if (dist < bloom_range) {
	// This bloom was sampled, we are done
	m_blooms_sampled_labels.insert(bloom_label);
	
	it2->set_color("edge", "green");
	it2->set_color("fill", "green");
	Notify("VIEW_CIRCLE", it2->get_spec());
	
	//break; 
      }
      
    } // end of for loop through all blooms
    
  } // end of loop through node records
  int count_int =0;
  return(true);
}

double FldBloomStormSim::getRange(const XYCircle & pulse, double time)
{
  return(pulse.getRad());
  // below is for range pulses
  double range = 0.0;
  double elapsed_time = time - pulse.get_time();
  double radius = pulse.getRad();
  double duration = pulse.get_duration();

  if (elapsed_time < 0.0)
    return(0.0); 

  double pct = 1.0;
  if (duration > 0)
    pct = elapsed_time / duration;

  if (pct > 1)
    pct = 1;

  range = pct * radius;

  return(range);
}


//------------------------------------------------------------
// Procedure: postEval()
void FldBloomStormSim::postEval()
{
  if (!m_deployed)
    return;
  
  double num_agents = 0;
  double num_exploring = 0;
  std::map<std::string, std::string>::iterator it;
  for (it =  m_vechicle_opt_status.begin(); it != m_vechicle_opt_status.end(); it++){
    std::string vname = it->first;
    std::string option_name = it->second;
    
    // do we have a node report for this agent?
    if (m_map_node_records.count(vname) > 0)
      num_agents += 1.0; 
    else
      continue;

    // is this agent exploring?
    if (option_name == "EXPLORE")
      num_exploring += 1.0;
  }

  // safety check:
  if (num_agents == 0.0)
    num_agents = 1.0;

  

  double ratio = num_exploring / num_agents;
  Notify("MIXED_STRAT", ratio);

  // update the running average;
  // if we just started, mark down the time
  // set the flags and return
  if (!m_eval_started){
    m_last_eval_time = MOOSTime();
    m_eval_started = true;
    return;
  }

  // otherwise compute the running average
  double dt = MOOSTime() - m_last_eval_time;

  double numerator = m_mixed_strat_run_avg * m_mixed_strat_total_running_time;
  numerator += ratio * dt;

  double denom = m_mixed_strat_total_running_time + dt;
  if (denom == 0.0)
    denom = 1.0;
  
  m_mixed_strat_run_avg = numerator / denom;
  m_last_eval_time = MOOSTime();
  m_mixed_strat_total_running_time += dt;
  Notify("MIXED_STRAT_RNG_AVG", m_mixed_strat_run_avg);
  
  
  return;
}


//------------------------------------------------------------
// Procedure: buildReport()

bool FldBloomStormSim::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: uFldBloomStormSim                     " << endl;
  m_msgs << "============================================" << endl;
  
  m_msgs << " Active Blooms: " << uintToString(m_active_blooms.size()) << "             " << endl;
  m_msgs << "============================================" << endl;
  std::vector<XYCircle>::iterator it;
  for (it = m_active_blooms.begin(); it != m_active_blooms.end(); it++){
    m_msgs << it->get_spec() << endl;
  }

  m_msgs << "                                            " << endl;
  m_msgs << " Node Reports : " << uintToString(m_map_node_records.size()) << "   ";
 
  
  std::string bloom_list_str = " Vehicles in bloom: ";
  std::set<std::string>::iterator it2;
  for (it2 = m_vehicles_in_bloom.begin(); it2 != m_vehicles_in_bloom.end(); it2++) {
    bloom_list_str += " " + *it2 + ",";
  }
  if ( m_vehicles_in_bloom.size() > 0){
      // at least one agent is in the set, remove the last comma
    bloom_list_str = bloom_list_str.substr(0, bloom_list_str.size()-1);
  }
  m_msgs << bloom_list_str << endl;

  m_msgs << " Completed Blooms:                         " << endl;
  m_msgs << "                Sampled: " << uintToString(m_sampled_blooms_completed) << "        " << endl;
  m_msgs << "          Only Detected: " << uintToString(m_detected_blooms_completed) << "        " << endl;
  m_msgs << " Unsampled & Undetected: " << uintToString(m_undetected_and_unsampled_blooms_completed) << "        " << endl;
 
  m_msgs << "                                            " << endl;
  m_msgs << "============================================" << endl;
  if (m_storm_active) {
    m_msgs << " Active Storm: " << m_storm_circle.get_spec() << endl;

    std::string storm_list_str = "  *Vehicles in storm: ";
    std::set<std::string>::iterator it3;
    for (it3 = m_vehicles_in_storm.begin(); it3 != m_vehicles_in_storm.end(); it3++) {
      storm_list_str += " " + *it3 + ",";
    }
    if ( m_vehicles_in_storm.size() > 0){
      // at least one agent is in the set, remove the last comma
      storm_list_str = storm_list_str.substr(0, storm_list_str.size()-1);
    }
    m_msgs << storm_list_str << endl;

  }
  
    
  return(true);
}




