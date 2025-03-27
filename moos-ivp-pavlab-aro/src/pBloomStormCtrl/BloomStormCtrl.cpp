/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BloomStormCtrl.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "BloomStormCtrl.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

BloomStormCtrl::BloomStormCtrl()
{
  m_curr_option = "";
  m_storm_detected = false;
  m_bloom_detected = false;
  m_new_bloom_detected = false;
  m_no_waypoints_available_to_sample = true; 
  
  m_nav_x = -1;
  m_nav_y = -1;
  m_nav_hdg = 0.0; 
  m_odometry = 0.0;

  m_send_new_ignore_set = false;

  m_min_batt = 11.5;
  m_max_batt = 15.5;
  m_battery_exhausted = m_max_batt;
  m_time_const_batt = 10.0;
  m_batt_exh_gain = 50; 
  
  m_m300_batt_voltage = 0;
  m_got_real_batt_voltage = false;
  m_current_batt_voltage = 0.0;

  m_sample_radius = 15;
  m_sample_time   = 30;
  m_sample_capture_radius = 5;
  m_fixed_coalitions = false; 

  m_migration_done = false;
  m_finish_migration_val = 80;
  
  m_next_region = false;   // false = region2
  m_region_1_spec = "-300,-250:-28.9,-121.5:121,-437.8:-171.5,-566.3";
  m_region_2_spec = "125,-50:394.1,78.5:546,-237.8:253.5,-366.3";
  
  m_bloom_found_counter = 0;


  //m_dist_to_closest_sample = 1000;

  m_no_sample_available_weight = 50;
  m_currently_sampling_weight  = 50;
  m_number_sampled = 0;

  m_explore_bias = 0.0;
  m_all_input_gain = 0.1;
  m_search_value_improvement_gain = 0.01;

  m_storm_detected_input_val = 100;
  m_finish_migration_val = 60;
  m_time_since_last_migration_thresh = 300;
  m_time_since_last_migration_resistance_val = -100;
  m_time_since_no_samples_thresh = 60;
  m_time_since_no_samples_encouragement_slope = 0.1;


  m_value_with_participation  = 0.0;
  m_value_with_participation_time = 0.0;
  m_value_without_participation = 0.0;
  m_value_without_participation_time = 0.0;
  
  m_cost_with_participation  = 0.0;
  m_cost_with_participation_time = 0.0;
  m_cost_without_participation = 0.0;
  m_cost_without_participation_time = 0.0;

  m_own_target = "to-be-set";
  m_own_backup_target_spec = "to-be-set";
  m_own_target_time = 0.0;
  m_own_target_priority = 0.0;
  m_own_target_priority_time = 0.0;
  


}

//---------------------------------------------------------
// Destructor

BloomStormCtrl::~BloomStormCtrl()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool BloomStormCtrl::OnNewMail(MOOSMSG_LIST &NewMail)
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

    std::cout << " On New Mail, key = " << key << std::endl;
    
    bool handled = false; 
    if((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL")) {
      handled = handleMailNodeReport(msg.GetString());
      if (!handled)
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "NAV_X") {
      m_nav_x = msg.GetDouble();
      m_nav_x_vec.push_back( msg.GetDouble() ); 

    } else if (key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
      m_nav_y_vec.push_back( msg.GetDouble() );

    } else if (key == "NAV_HEADING") {
      m_nav_hdg= msg.GetDouble();

    } else if (key == "CONTACTS_LIST") {
        // parse message
      std::vector<std::string> svector = parseString(msg.GetString(), ',');
      for (unsigned int i=0; i<svector.size(); i++) {
	m_contacts.insert(svector[i]);
      }
      m_send_new_ignore_set = true;

    } else if (key == "EXPLORE_LIST") {
      // Make a new set to erase the old set.
      std::set<std::string> new_voronoi_active_set;
        // parse message
      std::vector<std::string> svector = parseString(msg.GetString(), ',');
      for (unsigned int i=0; i<svector.size(); i++) {
	new_voronoi_active_set.insert(svector[i]);
      }
      // only update if not in fixed coalitions
      if(!m_fixed_coalitions)
	m_voronoi_active = new_voronoi_active_set;
      
      m_send_new_ignore_set = true;

    } else if (key == "EXPLOIT_LIST") {
      // Make a new set to erase the old set.
      std::set<std::string> new_sample_active_set;
        // parse message
      std::vector<std::string> svector = parseString(msg.GetString(), ',');
      for (unsigned int i=0; i<svector.size(); i++) {
	new_sample_active_set.insert(svector[i]);
      }
      // add ourselves if exploiting
      if (m_curr_option == "EXPLOIT")
	new_sample_active_set.insert(m_vname); 
      // only update if not in fixed coalitions
      if(!m_fixed_coalitions)
	m_sample_active = new_sample_active_set;

    } else if (key == "M300_BATT_VOLTAGE") {
      m_m300_batt_voltage = msg.GetDouble();
      m_got_real_batt_voltage = true;
      m_time_last_got_batt = MOOSTime(); 

    } else if (key == "BLOOM_DETECTED") {
      handled = handleBloomMsg(msg.GetString());
      if (!handled) {
	reportRunWarning("Unhandled Mail: " + key);
      }
    } else if (key == "STORM_DETECTED") {
      handled = setBooleanOnString(m_storm_detected, msg.GetString());
      if (!handled) {
	reportRunWarning("Unhandled Mail: " + key);
      }
    } else if (key == "OPTION") {
      handled =  handleOptionMsg(msg.GetString());
      if (!handled) {
	reportRunWarning("Unhandled Mail: " + key);
      }
    } else if (key == "SAMPLE_LOC") {
      XYPoint new_point = string2Point(msg.GetString()); 
      if (new_point.valid()){
	m_bloom_records.push_back(new_point); 
      } else {
	reportRunWarning("Unhandled Mail: " + key + " Bad Point");
      }
      std::cout << " Just got mail for Sample LOC = " << msg.GetString() << std::endl;

    } else if (key == "SAMPLE_STARTING") {
      m_map_sampling_agents[msg.GetString()] = MOOSTime();

    } else if (key == "SAMPLE_FINISHING") {
      std::map<std::string, double>::iterator iter = m_map_sampling_agents.find(msg.GetString()); 
      if ( iter != m_map_sampling_agents.end()){
      	m_map_sampling_agents.erase(iter);
	if (msg.GetString() == m_vname){
	  m_number_sampled += 1;
	  Notify("NUMBER_OF_SAMPLED_BLOOMS", uintToString(m_number_sampled)); 
	}
      }
    } else if (key == "MIGRATION_DONE") {
      handled = setBooleanOnString(m_migration_done, msg.GetString());
      if (!handled) {
	reportRunWarning("Unhandled Mail: " + key);
      } else if (m_migration_done) {
	m_time_of_last_migration_completion = msg.GetTime(); 
      } else {
	m_time_of_last_migration_started = msg.GetTime(); 
      }

      //    New coms with pGroupComboAlloc
      
    } else if (key == "OWN_TARGET"){
      if (msg.GetString() != ""){
	m_own_target = msg.GetString();
	m_own_target_time = msg.GetTime();
      } else
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "OWN_TARGET_PRIORITY"){
      handled = setDoubleOnString(m_own_target_priority, msg.GetString());
      if (!handled) 
	reportRunWarning("Unhandled Mail: " + key);
      else
	m_own_target_priority_time = msg.GetTime();

    } else if (key == "COST_WITH_PARTICIPATION") {
      m_cost_with_participation = msg.GetDouble();
      m_cost_with_participation_time = msg.GetTime();

    } else if (key == "COST_WITHOUT_PARTICIPATION") {
      m_cost_without_participation = msg.GetDouble();
      m_cost_without_participation_time = msg.GetTime();

      // New coms with ProxonoiGridSearch
    } else if (key == "VALUE_WITH_PARTICIPATION") {
      m_value_with_participation = msg.GetDouble();
      m_value_with_participation_time = msg.GetTime();

    } else if (key == "VALUE_WITHOUT_PARTICIPATION") {
      m_value_without_participation = msg.GetDouble();
      m_value_without_participation_time = msg.GetTime(); 

    } else if(key != "APPCAST_REQ") {// handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
    }
   }

  std::cout << " Exiting On New Mail: "  << std::endl;
    
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool BloomStormCtrl::OnConnectToServer()
{
   registerVariables();
   Notify("NUMBER_OF_SAMPLED_BLOOMS", uintToString(0));
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool BloomStormCtrl::Iterate()
{
 
  std::cout << " ----------- Iterating --------------- " << std::endl;

  AppCastingMOOSApp::Iterate();
  std::cout << " ----------- After AppCastingMOOSApp::Iterate--------------- " << std::endl;
  
  postDiscovery();
  prepToDetermineSamplePoint();
  updateSamplePoint();
  
  calculateInputs(MOOSTime());
  publishInputs();

  if ( m_send_new_ignore_set) {
    publishIgnoreLists(); 
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool BloomStormCtrl::OnStartUp()
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
    
    if(param == "region_1_spec") {
      m_region_1_spec = value;
      m_region_1_poly = string2Poly(value);
      m_region_1_poly.grow_by_pct(1.05); 
      handled = m_region_1_poly.is_convex();
    }
    else if(param == "region_2_spec") {
      m_region_2_spec = value; 
      m_region_2_poly = string2Poly(value);
      m_region_2_poly.grow_by_pct(1.05); 
      handled = m_region_2_poly.is_convex();
    }
    else if(param == "sample_radius") {
      handled = setDoubleOnString(m_sample_radius, value);
    } else if(param == "sample_time") {
      handled = setDoubleOnString(m_sample_time, value);
    } else if(param == "max_batt") {
      handled = setPosDoubleOnString(m_max_batt, value);
    } else if(param == "min_batt") {
      handled = setPosDoubleOnString(m_min_batt, value);
    } else if(param == "sample_capture_radius") {
      handled = setDoubleOnString(m_sample_capture_radius, value);
    } else if(param == "no_sample_available_weight") {
      handled = setDoubleOnString(m_no_sample_available_weight, value);
    } else if(param == "all_input_gain") {
      handled = setDoubleOnString(m_all_input_gain, value);
    } else if(param == "search_value_improvement_gain") {
      handled = setDoubleOnString(m_search_value_improvement_gain, value);
    } else if(param == "batt_exh_gain") {
      handled = setDoubleOnString(m_batt_exh_gain, value);
    } else if(param == "explore_bias") {
      handled = setDoubleOnString(m_explore_bias, value); 
    } else if(param == "currently_sampling_weight") {
      handled = setDoubleOnString(m_currently_sampling_weight, value);
    } else if(param == "storm_detected_input_val") {
      handled = setDoubleOnString(m_storm_detected_input_val, value);
    } else if(param == "finish_migration_input_val") {
      handled = setDoubleOnString(m_finish_migration_val, value);
    } else if(param == "time_since_last_migration_thresh") {
      handled = setDoubleOnString(m_time_since_last_migration_thresh, value);
    } else if(param == "time_since_last_migration_resistance_val") {
      handled = setDoubleOnString(m_time_since_last_migration_resistance_val, value);
    } else if(param == "time_since_no_samples_thresh") {
      handled = setDoubleOnString(m_time_since_no_samples_thresh, value);
    } else if(param == "time_since_no_samples_encouragement_slope") {
      handled = setDoubleOnString(m_time_since_no_samples_encouragement_slope, value);
    } else if(param == "cutoff_freq_batt") {
      double cutoff;
      bool ok = setPosDoubleOnString(cutoff,value);
      m_time_const_batt = 1.0 / cutoff;  // Cutoff cannot be zero.
      handled = ok;
    }

    // Taken from opinion fixer for fixed coalition testing
    else if(param == "persistent_exploit_set") {
      handled = true;
      m_fixed_coalitions = true; 
      vector<string> msgs = parseString(value, ',');
      for(unsigned int i=0; i<msgs.size(); i++) {
	string msg = stripBlankEnds(msgs[i]);
	if(msg != "")
	  m_sample_active.insert(msg);
	else
	  handled = false;
      }
      

    }
     else if(param == "persistent_explore_set") {
      handled = true;
      m_fixed_coalitions = true; 
      vector<string> msgs = parseString(value, ',');
      for(unsigned int i=0; i<msgs.size(); i++) {
	string msg = stripBlankEnds(msgs[i]);
	if(msg != "")
	  m_voronoi_active.insert(msg);
	else
	  handled = false;
      }

    }


    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  // Get vname from .MOOS Mission File
  bool vnameOK = m_MissionReader.GetValue("Community", m_vname);
  if(!vnameOK){
    reportConfigWarning("Vehicle name missing in MOOS file.");
    return(false);
  }

  // Post vals for the log file
  Notify("EXPLORE_BIAS", m_explore_bias); 
  Notify("SEARCH_VALUE_IMPRV_GAIN", m_search_value_improvement_gain);
  Notify("BATT_EXH_GAIN", m_batt_exh_gain);
  Notify("ALL_INPUT_GAIN", m_all_input_gain);

  m_time_of_last_migration_started = MOOSTime(); 
  m_time_of_last_migration_completion = MOOSTime();
  m_time_of_last_sample_available = MOOSTime();
  
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void BloomStormCtrl::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT",0);
  Register("NODE_REPORT_LOCAL",0);
  Register("OPTION",0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING",0); 
  Register("CONTACTS_LIST", 0);
  // To uncomment
  Register("EXPLORE_LIST", 0);
  Register("EXPLOIT_LIST",0);
  Register("M300_BATT_VOLTAGE",0);
  Register("BLOOM_DETECTED",0);
  Register("STORM_DETECTED",0);
  Register("SAMPLE_LOC",0);
  Register("SAMPLE_FINISHING",0);
  Register("SAMPLE_STARTING",0);
  Register("MIGRATION_DONE",0);

  Register("VALUE_WITH_PARTICIPATION",0);
  Register("VALUE_WITHOUT_PARTICIPATION",0);
  Register("COST_WITH_PARTICIPATION",0);
  Register("COST_WITHOUT_PARTICIPATION",0);

  Register("OWN_TARGET", 0);
  Register("OWN_TARGET_PRIORITY", 0);
 
}


//------------------------------------------------------------
// Procedure: buildReport()

bool BloomStormCtrl::buildReport() 
{
  
 ACTable actab2(4);
  actab2 << " Current Option |  Odometry | Batt Volts | Samples Found ";
  actab2.addHeaderLines();
  actab2 << m_curr_option << doubleToString(m_odometry) << doubleToString(m_current_batt_voltage) <<  uintToString(m_bloom_found_counter); 
  m_msgs << actab2.getFormattedString();
  m_msgs << endl;

  
  if (m_map_sampling_agents.find(m_vname) != m_map_sampling_agents.end()) {
    m_msgs << "                                                           " << endl;
    m_msgs << " Sampling now for : " << doubleToString(MOOSTime() - m_map_sampling_agents[m_vname]) << std::endl; 
  }

  double dt_no_samples = MOOSTime() - m_time_of_last_sample_available;
  m_msgs << "   Debug: time since no samples avail = " << doubleToString(dt_no_samples,4) << endl;
  m_msgs << "   Debug:  m_time_of_last_sample_available = " << doubleToString( m_time_of_last_sample_available,4) << endl;
  m_msgs << "                                                           " << endl;
  m_msgs << "=  Input Values  ==========================================" << endl;
  m_msgs << "===========================================================" << endl;
  std::map<std::string, double>::iterator it;
  for (it = m_opinion_inputs.begin(); it!= m_opinion_inputs.end(); it++) {
    m_msgs << "  " << it->first << ": " <<  doubleToStringX(it->second, 4) << endl; 
  }

  m_msgs << "                                                           " << endl;
  m_msgs << "=  Sample Points: " << "(" << uintToString(m_bloom_records.size()) << ")           " << endl;
  m_msgs << "===========================================================" << endl;
  std::list<XYPoint>::iterator it2;
  for (it2 = m_bloom_records.begin(); it2!= m_bloom_records.end(); it2++) {
    m_msgs << "  *" << it2->get_spec() << endl; 
  }


  return(true);
}


//----------------------------------------------------------
// Procedure: postDiscovery();
//            Post if discovered a new bloom
void BloomStormCtrl::postDiscovery()
{
  std::cout << "   In postDiscovery()         " << std::endl;
  // if we are not in the right mode, then return
  if ( m_curr_option != "EXPLORE")
    return;

  // if we are not far enough away from any known blooms then return
  std::list<XYPoint>::iterator it;
  double dx, dy, dist;
  for ( it = m_bloom_records.begin(); it != m_bloom_records.end(); it++) {
    dx = m_nav_x - it->get_vx();
    dy = m_nav_y - it->get_vy();
    dist = sqrt( dx * dx + dy * dy);
    if (dist < 2 * m_sample_radius)
      return;
  }

   
  bool post_discovery = false; 
  // check if we just switched to explore from another mode, and we now
  // detect a bloom.
  // Commented out because in the new pOpinionManager only the one posting
  // is made so last_option is always different then current option
  //bool cond1 = (m_last_option != m_curr_option);
  //bool cond2 = (m_curr_option == "EXPLORE");

  //if (cond1 && cond2 && m_bloom_detected)
  //  post_discovery = true;

  if (m_new_bloom_detected)
    post_discovery = true;

  if (post_discovery) {
    m_bloom_found_counter += 1;
    
    // Create four points:
    std::string preamble = m_vname + "_" + uintToString(m_bloom_found_counter);
    
    double right_x = m_nav_x + m_sample_radius * cos(m_nav_hdg * M_PI/180.0);
    double right_y = m_nav_y - m_sample_radius * sin(m_nav_hdg * M_PI/180.0);
    XYPoint right(right_x, right_y, preamble + "_right");

    double left_x = m_nav_x - m_sample_radius * cos(m_nav_hdg * M_PI/180.0);
    double left_y = m_nav_y + m_sample_radius * sin(m_nav_hdg * M_PI/180.0);
    XYPoint left(left_x, left_y, preamble + "_left");

    double ahead_x = m_nav_x + m_sample_radius * sin(m_nav_hdg * M_PI/180.0);
    double ahead_y = m_nav_y + m_sample_radius * cos(m_nav_hdg * M_PI/180.0);
    XYPoint ahead(ahead_x, ahead_y, preamble + "_ahead");

    XYPoint center(m_nav_x, m_nav_y, preamble + "_cent");
    
     
    right.set_color("vertex", "yellow");
    left.set_color("vertex", "yellow");
    ahead.set_color("vertex", "yellow");
    center.set_color("vertex", "yellow");
    
    // Post the four points:
    NodeMessage node_message;
    node_message.setSourceNode(m_vname);
    node_message.setDestNode("all");
    node_message.setVarName("SAMPLE_LOC");

    node_message.setStringVal(right.get_spec());
    Notify("NODE_MESSAGE_LOCAL", node_message.getSpec());
    Notify("SAMPLE_LOC", right.get_spec());
    
    node_message.setStringVal(left.get_spec());
    Notify("NODE_MESSAGE_LOCAL", node_message.getSpec());
    Notify("SAMPLE_LOC", left.get_spec());
    
    node_message.setStringVal(ahead.get_spec());
    Notify("NODE_MESSAGE_LOCAL", node_message.getSpec());
    Notify("SAMPLE_LOC", ahead.get_spec());
    
    node_message.setStringVal(center.get_spec());
    Notify("NODE_MESSAGE_LOCAL", node_message.getSpec());
    Notify("SAMPLE_LOC", center.get_spec());
    
    // Add them to the records
    //m_bloom_records.push_back(north);
    //m_bloom_records.push_back(east);
    //m_bloom_records.push_back(south);
    //m_bloom_records.push_back(west);
    //m_bloom_records.push_back(center);

    // Publish them to view
    Notify("VIEW_POINT", right.get_spec());
    Notify("VIEW_POINT", left.get_spec());
    Notify("VIEW_POINT", ahead.get_spec());
    Notify("VIEW_POINT", center.get_spec());
    
    m_new_bloom_detected = false; 
  }
    
  
  return;
}


//----------------------------------------------------------
// Procedure: getExtraCost(vname);
//            
double BloomStormCtrl::getExtraCost(std::string agent_name)
{
  double extra_cost = 0.0;
  std::map<std::string, double>::iterator it_sa;
  it_sa = m_map_sampling_agents.find(agent_name); 
  if ( it_sa != m_map_sampling_agents.end()){
    extra_cost = m_sample_time - (MOOSTime() - m_map_sampling_agents[agent_name]);
    if (extra_cost < 0.0)
      extra_cost = 0.0;
  } else {
    extra_cost = 0.0;
  }

  return(extra_cost);
}

//----------------------------------------------------------
// Procedure: prepToDetermineSamplePoint();
//            Bookkeeping and prep to configure pGroupComboAlloc
//            To solve the multi-agent TSP
void BloomStormCtrl::prepToDetermineSamplePoint()
{


  std::cout << " UpdatingSamplePoints()" << std::endl;
  // Step 1:
  // First check if others who are sampling have reached any sample
  // point and remove it from the list.
  std::map<std::string, NodeRecord>::iterator it_nr;
  std::list<XYPoint>::iterator it_br;
  double dist; 

  // also get the spec of the closest sample to ownship as a backup
  // in case the TSP solver is slow
  double min_dist = 100000000.0;
  m_own_backup_target_spec = "to-be-set";
  
  std::cout << "  Checking status of sampling and building vector " << std::endl;
  std::cout << "   ===> m_sample_active = " << stringSetToString(m_sample_active) << std::endl;
  for (it_nr = m_map_node_records.begin(); it_nr != m_map_node_records.end(); it_nr++ ) {
    // continue if this vehicle is not actively sampling
    std::cout << "  -Checking vehicle: " << it_nr->second.getName() << std::endl;
    if (m_sample_active.count(it_nr->second.getName())== 0) {
      std::cout << "  --Continuing " << std::endl;
      continue; 
    }

    for (it_br = m_bloom_records.begin(); it_br != m_bloom_records.end();  ) {
      std::cout << "  -- Checking bloom = " << it_br->get_spec() << std::endl;
      dist = pow( (it_nr->second.getX() - it_br->get_vx()), 2) +  pow( (it_nr->second.getY() - it_br->get_vy()), 2);
      std::cout << "  --dist "<< doubleToStringX(dist,4) << std::endl;

      // remove this if it is captured and we've recieved a message from this agent
      // that they have started sampling
      bool sample_point_captured = dist < m_sample_capture_radius * m_sample_capture_radius;
      //bool agent_sampling = m_sample_active.count(it_nr->second.getName());
      bool agent_sampling = m_map_sampling_agents.find(it_nr->second.getName()) != m_map_sampling_agents.end(); 
      
      if ( sample_point_captured && agent_sampling ){
	// Assume this is done, and remove it from the list
	// and clear it for pMarineViewer
	// and break since the vehicle can only be sampling at one time.
	it_br->set_active(false);
        Notify("VIEW_POINT", it_br->get_spec());
	m_bloom_records.erase(it_br++);
	std::cout << "  ---Found vehicle is sampling here, removing from the list" << doubleToStringX(dist,4) << std::endl;
	//break;  // found this agent is sampling this bloom, no need to check other blooms
	
      } else {
	// check if this is ownship
	bool is_ownship = (m_vname == it_nr->second.getName());
	if ( (is_ownship) && (dist < min_dist) ){
	  // we found a bloom that is closer
	  m_own_backup_target_spec = it_br->get_spec_xy(',');
	  min_dist = dist; 
	}
	++it_br;	
      }
    }
  }
  // At this point we have updated m_bloom_records;
  
  // Step 2:
  // Complete a quick sanity check of the vehicles which have reported that they are sampling
  // This clears out old values in case of communication droppout.
  
  std::map<std::string, double>::iterator it_sa;
  for (it_sa = m_map_sampling_agents.begin(); it_sa != m_map_sampling_agents.end();) {
    if ((MOOSTime() - it_sa->second) > m_sample_time) {
      it_sa = m_map_sampling_agents.erase(it_sa);
    } else {
      ++it_sa;
    }
  }

  // Step 3:
  // Now pick the next sample pont.
  // This will be handled by the pGroupComboAlloc app, which uses the hungarian algo
  // for optimality.
  
  // This app needs to send:
  // 1) Sample List
  // pGroupComboAlloc is already listening for SAMPLE_LOC and agents that are exploiting
  // 2) additional cost for each agent, in this case the time remaining to compelete the sample
  //

  // Send sample list
  std::string sample_list_spec = "";
  for (it_br = m_bloom_records.begin(); it_br != m_bloom_records.end(); it_br++) {
    sample_list_spec += it_br->get_label() + ",";
  }
  
  if (m_bloom_records.size() > 0){
    sample_list_spec = sample_list_spec.substr(0,sample_list_spec.size()-1);
  }
  Notify("SAMPLE_LIST", sample_list_spec);


  // Send extra costs
  // Extra cost, agent:value
  // Reuse iterators it_nr and it_sa
  double extra_cost = 0.0;
  for (it_nr = m_map_node_records.begin(); it_nr != m_map_node_records.end(); it_nr++ ) {
    // is this agent sampling?
    std::string agent_name = it_nr->second.getName();
    
    extra_cost = getExtraCost(agent_name);
    Notify("EXTRA_COST", agent_name + ":" + doubleToStringX(extra_cost,3));   
  }

  // And own extra cost
  extra_cost = getExtraCost(m_vname);
  Notify("EXTRA_COST", m_vname + ":" + doubleToStringX(extra_cost,3));   
  
  
  // And will recieve:
  // If participating:
  // 1) OWN_TARGET, ex sample_1
  // 2) OWN_TARGET_PRIORITY, ex, 1,2,3 ...
  // 3) UNASSIGNED_SAMPLE, ex true, false

  // If configured we should also get:
  // 4) COST_WITH_PARTIPICATION
  // 5) COST_WITHOUT_PARTICIPATION
  

  // These actions are handled in other functions,
  // updateSamplePoints, calcInputs


  return;
}

//----------------------------------------------------------
// Procedure:  updateSamplePoint()
void BloomStormCtrl::updateSamplePoint()
{
  // From pGroupComboAlloc, we recieve:
  // If participating:
  // 1) OWN_TARGET, ex sample_1
  // 2) OWN_TARGET_PRIORITY, ex, 1,2,3 ... This should never be greater than 1
  //                         in the sampling case. If there are more agents than
  //                         samples, some agents will not be allocated.  Those
  //                          agents will see the cost with participating is the same
  //                         or worse than without.
  

  // If configured we should also get:
  // 4) COST_WITH_PARTIPICATION
  // 5) COST_WITHOUT_PARTICIPATION

  // if I didn't get a target, where the marginal improvement of my participation
  // is zero or negative, then I should just use backup target,
  // or the default centers of the region to update the waypoint behavior
  std::string spec;
  if(m_own_backup_target_spec != "to-be-set"){
    spec = m_own_backup_target_spec; 
  } else {
    if (m_next_region) {
      // region 1 is next, region 2 is active
      spec = "325,-110";
    } else {
      // region 2 is next, region 1 is active
      spec= "-100,-310";  
    }
  }

  bool fresh_target_data = isTargetInfoFresh();
  double cost_improvement = m_cost_without_participation - m_cost_with_participation;

  if ((fresh_target_data) && (cost_improvement > 0.0)){
    m_no_waypoints_available_to_sample = false;
    // update waypoint behavior(s)
    // Find spec with the label m_own_target
    std::list<XYPoint>::iterator it_idx;
    for (it_idx = m_bloom_records.begin(); it_idx != m_bloom_records.end(); it_idx++){
      if (it_idx->get_label() == m_own_target){
	spec = it_idx->get_spec_xy(',');
	break;
      }
    }
    m_time_of_last_sample_available = MOOSTime();

  } else {
    m_no_waypoints_available_to_sample = true;    
  }

  // Spec will be the default center of region, unless a bloom was found
  Notify("SAMPLE_TRAVEL_UPDATE","point=" + spec); 
  Notify("SAMPLE_UPDATE","station_pt=" + spec);

  return;

}


//---------------------------------------------------------
// Procedure: isTargetInfoFresh()
bool BloomStormCtrl::isTargetInfoFresh()
{
  double thresh = 60.0;

  if ((MOOSTime() - m_own_target_time) > thresh) 
    return(false);
  if ((MOOSTime() - m_own_target_priority_time) > thresh) 
    return(false);
  if ((MOOSTime() - m_cost_with_participation_time) > thresh) 
    return(false);
  if ((MOOSTime() - m_cost_without_participation_time) > thresh) 
    return(false);
  
  return(true);
}


//---------------------------------------------------------
// Procedure: isSearchValueInfoFresh()
bool BloomStormCtrl::isSearchValueInfoFresh()
{
  double thresh = 2.0;

  if ((MOOSTime() - m_value_with_participation_time) > thresh) 
    return(false);
  if ((MOOSTime() - m_value_without_participation_time) > thresh) 
    return(false);
  
  return(true);
}



//----------------------------------------------------------
// Procedure:  clearSamplePoints()
void BloomStormCtrl::clearSamplePoints()
{
  std::list<XYPoint>::iterator it_br;
  for (it_br = m_bloom_records.begin(); it_br != m_bloom_records.end(); ) {
    it_br->set_active(false);
    Notify("VIEW_POINT", it_br->get_spec());
    it_br = m_bloom_records.erase(it_br);
  }
  return;
}



//----------------------------------------------------------
// Procedure: calculateInputs(time)

bool BloomStormCtrl::calculateInputs(double time)
{

  // Calculate battery amount exhausted
  updateOdometry();
  double max_heron_range = 1 * 60 * 60 * 2;
  if (m_got_real_batt_voltage) {
    m_current_batt_voltage = m_m300_batt_voltage;
  } else {
    m_current_batt_voltage = (m_min_batt - m_max_batt) * (m_odometry / max_heron_range) + m_max_batt;
  }

  if (m_current_batt_voltage > m_max_batt)
    m_current_batt_voltage = m_max_batt;

  if (m_current_batt_voltage < m_min_batt)
    m_current_batt_voltage = m_min_batt; 

  double batt_state = (m_max_batt - m_current_batt_voltage)/ (m_max_batt - m_min_batt);

  double delta_t = MOOSTime() - m_time_last_got_batt;
  double alpha;
  
  // Low pass filter. 
  alpha = 1.0 - exp( -1.0 * delta_t / m_time_const_batt);
  m_battery_exhausted += alpha * ( batt_state - m_battery_exhausted); 

  if (!m_got_real_batt_voltage){
    m_time_last_got_batt = MOOSTime(); 
  }
  

  ////////////////////////
  // Explore input

  double explore_input = 0.0;
  
  // Marginal search value increase
  bool fresh_search_value_data = isSearchValueInfoFresh();
  double value_improvement = m_value_with_participation - m_value_without_participation; 
  if (fresh_search_value_data){
    explore_input += value_improvement * m_search_value_improvement_gain;

  }

  // Add the battery input
  explore_input += m_batt_exh_gain * m_battery_exhausted;

  // Multiply all these estimates by the all_input_gain;
  explore_input *= m_all_input_gain;
  
  // Add a term if there are no waypoints to sample
  // and no storm is detected
  //if (m_no_waypoints_available_to_sample && !m_storm_detected)
  //  explore_input += m_no_sample_available_weight;

  // Finally add the bias

  explore_input += m_explore_bias;
  m_opinion_inputs["EXPLORE_INPUT"] = explore_input; 
  

  ///////////////////////
  // Exploit input
  
  double exploit_input = 0.0;

  //bool fresh_target_data = isTargetInfoFresh();
  // A positive cost improvement is good.  It is better to think of these things as
  // positive instead of negative costs, because we want the bias to increase when the total
  // cost is decreasing, or the cost is improving.
  
  //     cost_improvement = +2 =       10                -          8
  double cost_improvement = m_cost_without_participation - m_cost_with_participation;

  //if (fresh_target_data){
    
  // Marginal cost improvement
  // Only add input if there are waypoints to sample
  if (cost_improvement <= 0.0) 
    exploit_input += 0.0; 
  else 
    exploit_input += cost_improvement ;
  
  //}

  // Multiply by the all input gain
  exploit_input *= m_all_input_gain;
  

  // Check if we are currently sampling
  std::map<std::string, double>::iterator iter = m_map_sampling_agents.find(m_vname); 
  if (iter != m_map_sampling_agents.end()) {
    exploit_input += m_currently_sampling_weight; 
  }

  // Finally subtract the bias. 
  exploit_input -= m_explore_bias;
  m_opinion_inputs["EXPLOIT_INPUT"] = exploit_input; 

  ////////////////////////
  // Migrate input

  double migrate_input = 0.0;
  
  if (m_storm_detected) {
      migrate_input += m_storm_detected_input_val; 
  }

  if (!m_migration_done){

    // did all my neighbors make it?
    std::map<std::string, NodeRecord>::iterator it_nr;
    bool all_neighbors_migrated = true; 
    for (it_nr = m_map_node_records.begin(); it_nr != m_map_node_records.end(); it_nr++ ) {
      // if it is not in the contacts list, and is not ownship, then skip it. 
      if (!m_contacts.count(it_nr->second.getName()) && (it_nr->second.getName() != m_vname) ) {
	std::cout << " migration calc, rejecting nr = " << it_nr->second.getName() << std::endl;
	continue; 
      }
      std::cout << " migration calc, checking nr = " << it_nr->second.getName() << std::endl;
      // m_next_region = true => current region was 2, the next region is 1;
      // m_next_region = flase => current region was 1, the next region is 2;
      if (m_next_region) {
	if (!m_region_2_poly.contains(it_nr->second.getX(), it_nr->second.getY())){
	  all_neighbors_migrated = false;
	  std::cout << " found this one was not contained in poly2= " << std::endl;
	  break;
	}	  
      } else {
	if (!m_region_1_poly.contains(it_nr->second.getX(), it_nr->second.getY())){
	  all_neighbors_migrated = false;
	  std::cout << " found this one was not contained in poly1= " << std::endl;
	  break;
	}
      }  
    }

    if (!all_neighbors_migrated){
      std::cout << " not all migrated, adding output.  " << std::endl;
      migrate_input += m_finish_migration_val;
    }
    
   }
  
  // Did we just complete a migration?
  double dt_migrate = MOOSTime() - m_time_of_last_migration_completion;

  if (( 2.0 < dt_migrate) && ( dt_migrate < m_time_since_last_migration_thresh ) && !m_storm_detected){
    migrate_input += m_time_since_last_migration_resistance_val * ( 1.0 - dt_migrate/m_time_since_last_migration_thresh);
  }

 
  // Are there no samples detected here?
  double dt_no_samples = MOOSTime() - m_time_of_last_sample_available;
  if ( dt_no_samples > m_time_since_no_samples_thresh) {
    migrate_input += (dt_no_samples - m_time_since_no_samples_thresh) * m_time_since_no_samples_encouragement_slope; 
  }

  m_opinion_inputs["MIGRATE_INPUT"] = migrate_input; 

  return(true); 
}


 //----------------------------------------------------------
// Procedure: publishInputs()

bool BloomStormCtrl::publishInputs()
{

  std::map<std::string, double>::iterator it;
  for (it = m_opinion_inputs.begin(); it!= m_opinion_inputs.end(); it++) {
    Notify(it->first, it->second); 
  }
  
  return(true); 
}

 
//---------------------------------------------------------------
// publishIgnoreLists()
//         Any and all
bool BloomStormCtrl::publishIgnoreLists()
{

  std::string val = "";
  std::set<std::string>::iterator it;
  for (it = m_contacts.begin(); it != m_contacts.end(); it++){
    std::cout << "Checking to ignore Contact = " << *it << std::endl;
    if (m_voronoi_active.count(*it)==0) {
      std::cout << " set to ignore" << std::endl;
      val += *it + ",";
    }
  }

  std::cout << " voronoi active set ="; 
  for (it = m_voronoi_active.begin(); it != m_voronoi_active.end(); it++) {
    std::cout << *it << ", ";

  }
  std::cout << std::endl;

  // remove the last comma if needed
  if (val.length()>0)
    val = val.substr(0, val.size()-1);

  Notify("PROX_SET_IGNORE_LIST", val);
  m_send_new_ignore_set = false;
  
  
  return(true);
}

void BloomStormCtrl::updateOdometry()
{
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

    Notify("N_ODOM_BLOOM_CTRL", m_odometry); 
  }
  return;
}

//---------------------------------------------------------
// Procedure: handleMailNodeReport()
//   Example: NAME=alpha,TYPE=KAYAK,UTC_TIME=1267294386.51,
//            X=29.66,Y=-23.49,LAT=43.825089, LON=-70.330030, 
//            SPD=2.00, HDG=119.06,YAW=119.05677,DEPTH=0.00,     
//            LENGTH=4.0,MODE=ENGAGED

bool BloomStormCtrl::handleMailNodeReport(const string& node_report_str)
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

  return(true);
}


//---------------------------------------------------------
// Procedure: handleOptionMsg()

bool BloomStormCtrl::handleOptionMsg(const std::string& msg) {

  // update history
  m_last_option = toupper(m_curr_option); 
  m_curr_option = toupper(msg);

  // update sets
  if (m_curr_option == "EXPLOIT") {
    m_sample_active.insert(m_vname);
    Notify("TMP_DEBUG_SAMPLE_ACTIVE_SET", stringSetToString(m_sample_active));
  

  // handle bloom detection
  } else if (m_curr_option == "EXPLORE"){
    m_sample_active.erase(m_vname); 
    m_new_bloom_detected = false;
  

  // handle migration
  } else if (m_curr_option == "MIGRATE"){
    clearSamplePoints();
    if  ( (MOOSTime() - m_time_of_last_migration_started) < m_time_since_last_migration_thresh)
      return(true);
    
    // if this is a new migration, then post the updated region info
    if (m_last_option != "MIGRATE"){
      
      std::string spec;
      // m_next_region = true => current region is 2, the next region is 1;
      // m_next_region = flase => current region is 1, the next region is 2;
      if (m_next_region) {
	// region1
	spec = m_region_1_spec;
      } else {
	// region2
	spec = m_region_2_spec; 
      }
      
      Notify("PROX_UP_REGION", spec);
      Notify("UP_MOVE", "region=pts={" + spec + "},label=two"); 
      Notify("VECTOR_UPDATE", "op_region=pts={" + spec + "}");
      Notify("CURR_REGION","pts={" + spec + "}"); 
      Notify("OP_REGION_RESET", "true"); 
      
      // cycle the flag for the next migration.
      m_next_region = !m_next_region; 
      
    }

  }
  
  return(true); 
}

bool BloomStormCtrl::handleBloomMsg(const std::string& msg)
{
  bool handled = false; 
  // check if this is a new bloom
  // i.e. did we just go from true to false?
  bool this_msg = false;
  handled = setBooleanOnString(this_msg, msg);
  if (!handled)
    return(false); 

  
  bool cond1 = (m_bloom_detected ==false);
  bool cond2 = (this_msg ==true);
  bool cond3 = (m_curr_option == "EXPLORE");
  bool in_region = false;

  // m_next_region = true => current region is 2, the next region is 1;
  // m_next_region = flase => current region is 1, the next region is 2;
  if (m_next_region) {
    if (m_region_2_poly.contains(m_nav_x, m_nav_y)){
      in_region = true; 
    }	  
  } else {
    if (m_region_1_poly.contains(m_nav_x, m_nav_y)){
      in_region = true; 
    }
  } 
  
  if (cond1 && cond2 && cond3 && in_region){
    // this is a new bloom
    m_new_bloom_detected = true;
  }
  m_bloom_detected = this_msg;

  return(true);
}
