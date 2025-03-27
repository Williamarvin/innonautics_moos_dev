/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: AqctNodeCtrl.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "AqctNodeCtrl.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

AqctNodeCtrl::AqctNodeCtrl()
{

  m_nav_x = 0.0;
  m_nav_y = 0.0;
  m_nav_hdg = 0.0;
  m_vname = "";
  m_curr_option = "";
  
  m_dist_to_own_flag = 0.0; 
  m_agents_in_our_zone = 0.0;
  
  //m_keep_in_region_shrink_amt = 0.0;
  m_team_color = "not_set";
  m_team_size  = 2;
  m_we_grabbed_their_flag = false;
  m_they_grabbed_our_flag = false;

  m_name_of_our_teammate_who_grabbed_their_flag = "";
  m_name_of_the_opposing_player_who_grabbed_our_flag = "";

  m_name_of_closest_enemy_to_player_with_flag = "";
  m_dist_of_closest_enemy_to_player_with_flag = 100.0;
  m_best_trail_dist  = 0.0;
  m_best_trail_angle = 0.0;

  m_dist_to_own_flag_gain = 1.0;
  m_enemy_agent_in_home_zone_gain = 20.0;
  m_defend_bias = 0.0;

  //m_min_reallocate_interval = 5.0;
  //m_max_reallocate_interval = 10.0;
  //m_wait_to_resend_interval = 2.0;
  //m_number_of_tasks_sent = 0;

  //m_assigned_to_intercept = false;
  //m_assigned_contact = "";

  m_intercept_heading_penalty = 0.1;
  m_intercept_static_lead_dist  = 9.0;
  m_intercept_speed_extrap   = 3.0;  //seconds
  m_last_grabber_intercept_trail_range = 0.0;
  m_use_dynamic_speed_based_lead_dist = false;

  m_loiter_offset = 0.0;
  m_loiter_offset_angle = 0.0; 
  
}

//---------------------------------------------------------
// Destructor

AqctNodeCtrl::~AqctNodeCtrl()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool AqctNodeCtrl::OnNewMail(MOOSMSG_LIST &NewMail)
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

    } else if (key == "NAV_Y") {
      m_nav_y = msg.GetDouble();

    } else if (key == "NAV_HEADING") {
      m_nav_hdg = msg.GetDouble();

    } else if (key == "CONTACTS_LIST") {
      if ( !handleContactsList(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
      
	
    } else if (key == "TAGGED_VEHICLES") {
      if ( !handleTaggedList(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if ( (key == "ATTACK_LIST") || (key == "DEFEND_LIST") ) {
      if ( !handlePopStateList(key, msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "BLUE_FLAG_GRABBED") {
      bool msg_bool = false;
      bool ok = setBooleanOnString(msg_bool, msg.GetString());
      if (ok){
	if (m_team_color == "blue")
	  m_they_grabbed_our_flag = msg_bool;
	else if (m_team_color == "red")
	  m_we_grabbed_their_flag = msg_bool;
      } else 
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "RED_FLAG_GRABBED") {
      bool msg_bool = false;
      bool ok = setBooleanOnString(msg_bool, msg.GetString());
      if (ok){
	if (m_team_color == "red")
	  m_they_grabbed_our_flag = msg_bool;
	else if (m_team_color == "blue")
	  m_we_grabbed_their_flag = msg_bool;
      } else 
	reportRunWarning("Unhandled Mail: " + key);

    } else if (key == "FLAG_SUMMARY"){
      if ( !handleFlagSummary(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);

      /*
    } else if (key == "INTRUDER_TASKED") {
      if ( !handleIntruderTasked(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);

    } else if (key == "ASSIGNED_TO_INTERCEPT") {
      if ( !setBooleanOnString(m_assigned_to_intercept, msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
	   
    } else if (key == "ASSIGNED_CONTACT") {
      if ( m_contacts.count(msg.GetString() ) )
	m_assigned_contact = msg.GetString(); 
      else 
	reportRunWarning("Unhandled Mail: " + key);
      */
    } else if (key == "OPTION"){
      if (msg.GetString() != ""){
	m_curr_option = tolower(msg.GetString());
      } else
	reportRunWarning("Unhandled Mail: " + key);
		 
    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool AqctNodeCtrl::OnConnectToServer()
{
  //registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool AqctNodeCtrl::Iterate()
{
  AppCastingMOOSApp::Iterate();

  calculateInputs();
  publishInputs();
  updateBlockingBehavior();
  updateDefense();
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool AqctNodeCtrl::OnStartUp()
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
    if(param == "home_zone_poly") {
      m_home_zone_poly = string2Poly(value);
      m_home_zone_poly.set_label("home_zone_poly");
      handled = m_home_zone_poly.is_convex();

    } else if (param == "own_flag_location"){
      m_own_flag_pt = string2Point(value);
      handled = true;
      
    } else if (param == "near_loiter_pt"){
      m_near_loiter_pt = string2Point(value);
      handled = true;
      
    } else if (param == "far_loiter_pt"){
      m_far_loiter_pt = string2Point(value);
      handled = true;
      
    } else if (param == "rname"){
      handled = false;
      if (strContains(value, "red")){
	m_team_color = "red";
	handled = true;
      } else if (strContains(value, "blue")) {
	m_team_color = "blue";
	handled = true;
      } 
    } else if (param == "team_size") {
      handled = setUIntOnString(m_team_size, value);
      if (handled && ( m_team_size > 9)){
	handled = false;
      }
    } else if (param == "dist_to_own_flag_gain"){
      handled = setDoubleOnString(m_dist_to_own_flag_gain, value); 
    } else if (param == "enemy_agent_in_home_zone_gain"){
      handled = setDoubleOnString(m_enemy_agent_in_home_zone_gain, value);
      /*
    } else if (param == "min_reallocate_interval"){
      handled = setPosDoubleOnString(m_min_reallocate_interval, value);
    } else if (param == "max_reallocate_interval"){
      handled = setPosDoubleOnString(m_max_reallocate_interval, value);
    } else if (param == "task_wait_to_resend_interval"){
      handled = setPosDoubleOnString(m_wait_to_resend_interval, value);
      */
    } else if (param == "intercept_static_lead_dist"){
      handled = setPosDoubleOnString(m_intercept_static_lead_dist, value);
    } else if (param == "intercept_speed_extrap"){
      handled = setPosDoubleOnString(m_intercept_speed_extrap, value);
    } else if (param == "use_dynamic_speed_based_lead_dist"){
      handled = setBooleanOnString(m_use_dynamic_speed_based_lead_dist, value);
    } else if (param == "intercept_heading_penalty"){
      handled = setPosDoubleOnString(m_intercept_heading_penalty, value);
    } else if (param == "defend_bias"){
      handled = setDoubleOnString(m_defend_bias, value);
    } else if (param == "loiter_offset"){
      handled = setPosDoubleOnString(m_loiter_offset, value);
    } else if (param == "loiter_offset_angle"){
      handled = setPosDoubleOnString(m_loiter_offset_angle, value);
 
 
    }
      
    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  // The vehicle name is the host community
  m_vname = m_host_community; 

  // Fill out the rosters
  // This should be more general, but for now we assume the
  // roles are one-n where no indexes are skipped
  std::vector<std::string> ones {"one", "two", "three", "four", "five", "six", "seven", "eight", "nine"};

  std::string opposing_team_name;
  std::string own_team_name;
  
  for (unsigned int i = 0; i < m_team_size; i++) { // start at 0
    if(m_team_color == "blue"){
      opposing_team_name = "red_";
      own_team_name = "blue_";
    }
    else if(m_team_color == "red") {
      opposing_team_name = "blue_";
      own_team_name = "red_";
    }
    opposing_team_name += ones[i];
    own_team_name += ones[i]; 
    m_opposing_team_names.insert(opposing_team_name);
    m_own_team_names.insert(own_team_name);
  }

  /*
  if (strContains(m_vname, "one"))
    m_number_of_tasks_sent += 1000;
  else if (strContains(m_vname, "two"))
    m_number_of_tasks_sent += 1000;
  else if (strContains(m_vname, "three"))
    m_number_of_tasks_sent += 3000;
  */
  
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void AqctNodeCtrl::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
   Register("NODE_REPORT", 0);
   Register("NODE_REPORT_LOCAL", 0);
   Register("CONTACTS_LIST", 0);
   Register("TAGGED_VEHICLES"); 
   Register("ATTACK_LIST", 0);
   Register("DEFEND_LIST", 0);
   Register("OPTION",0);
   Register("NAV_X", 0);
   Register("NAV_Y", 0);
   Register("NAV_HEADING", 0);
   

   Register("RED_FLAG_GRABBED", 0);
   Register("BLUE_FLAG_GRABBED", 0);
   Register("FLAG_SUMMARY",0);
   Register("INTRUDER_TASKED",0);
   Register("ASSIGNED_TO_INTERCEPT", 0);
   Register("ASSIGNED_CONTACT", 0);
 
}

//----------------------------------------------------------
// Procedure: calculateInputs()

void AqctNodeCtrl::calculateInputs()
{

  // How close are we to our flag?
  m_dist_to_own_flag = distPointToPoint(m_nav_x, m_nav_y, m_own_flag_pt.get_vx(), m_own_flag_pt.get_vy());

  // How many enemy agents are in our zone?
  m_agents_in_our_zone = 0.0;   // double for math
  m_names_of_enemy_agents_in_our_zone.clear();
  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end(); it++){
    
    // is this record for the opposing team?
    if (m_opposing_team_names.count(it->first)){
      
      // Are they in our zone?
      if ( m_home_zone_poly.contains( it->second.getX(), it->second.getY() ) ) {
	
	// Are they not tagged?
	if (m_tagged_vehicles.count(it->second.getName()) == 0){
	  m_names_of_enemy_agents_in_our_zone.insert(it->second.getName());
	  m_agents_in_our_zone += 1.0;
	}
      }
    }
  }

  // Assemble function
  double defend_input = m_dist_to_own_flag_gain * m_dist_to_own_flag
                        + m_enemy_agent_in_home_zone_gain * m_agents_in_our_zone
                        + m_defend_bias;


  // add the tag penalty here

  
  m_opinion_inputs["DEFEND_INPUT"] = defend_input;
  m_opinion_inputs["ATTACK_INPUT"] = -defend_input;

  // Do some cleanup.  If an enemy agent is no longer in our zone, we need to remove the
  // tasking data the map to reassign

  /*
  // Now clear out any old node reports
  std::map<std::string, double>::iterator it2;
  for (it2 = m_time_to_reassign_intruder_map.begin(); it2 != m_time_to_reassign_intruder_map.end();){
    if ( m_names_of_enemy_agents_in_our_zone.count(it2->first) ) {
	++it2;
      } else {
	m_time_to_reassign_intruder_map.erase(it2++);
      }
  }
  */

  return;
}



//----------------------------------------------------------
// Procedure: updateBlockingBehavior()
//            This could be updated as the number of agents
//            increases.  Right now we only assume one agent
//            could be blocking. 

void AqctNodeCtrl::updateBlockingBehavior()
{
  // Does our team have the flag?
  if (!m_we_grabbed_their_flag)
    return;
  
  // paranoid sanity check
  if (m_node_rec_map.count(m_name_of_our_teammate_who_grabbed_their_flag) == 0){
    Notify("TMP_DEBUG", "EXIting early");
    return;
  }
  // Which is the closest opposing player?
  double protect_x = m_node_rec_map[m_name_of_our_teammate_who_grabbed_their_flag].getX();
  double protect_y = m_node_rec_map[m_name_of_our_teammate_who_grabbed_their_flag].getY();
  double close_enemy_x, close_enemy_y, dist_enemy;
  
  std::map<std::string, NodeRecord>::iterator it;
  double min_distance = std::numeric_limits<double>::max();
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end(); it++){
    // is this record for the opposing team?
    if (m_opposing_team_names.count(it->first)){
      close_enemy_x = it->second.getX();
      close_enemy_y = it->second.getY();
      
      dist_enemy = distPointToPoint(protect_x, protect_y, close_enemy_x, close_enemy_y); 
      if (dist_enemy < min_distance){
	m_name_of_closest_enemy_to_player_with_flag = it->second.getName();
	m_dist_of_closest_enemy_to_player_with_flag = dist_enemy;
	min_distance = dist_enemy;
      }
    }
  }
  
  // Calculate the geometry
  m_best_trail_dist = m_dist_of_closest_enemy_to_player_with_flag * 0.8;  //?
  
  // assume absoluate angle for trail behavior
  m_best_trail_angle = relAng(protect_x, protect_y, close_enemy_x, close_enemy_y); 
  
  // Send it
  Notify("TRAIL_INFO", "trail_range="+doubleToString(m_best_trail_dist));
  Notify("TRAIL_INFO", "trail_angle="+doubleToString(m_best_trail_angle));
  Notify("TRAIL_INFO", "contact="+m_name_of_our_teammate_who_grabbed_their_flag);
  
  return;
}

//----------------------------------------------------------
// Procedure: updateDefense()

void AqctNodeCtrl::updateDefense()
{

  // They grabbed our flag !!!
  // update the trail behavior if they grabbed our flag.
  if(m_they_grabbed_our_flag){
    // adjust trail range of intercept_grabber_behavior
    // Are we faurther away from the enemy with the tag?
    double enemy_x = m_node_rec_map[m_name_of_the_opposing_player_who_grabbed_our_flag].getX();
    double enemy_y = m_node_rec_map[m_name_of_the_opposing_player_who_grabbed_our_flag].getY();
    double dist_to_enemy = distPointToPoint(m_nav_x, m_nav_y, enemy_x, enemy_y);
    double new_trail_range;

    double max_trail_range = 35.0;
    double min_trail_range = 3.0;
    if (dist_to_enemy > 25.0)
      new_trail_range = max_trail_range;
    else if ((15 < dist_to_enemy) && (dist_to_enemy <= 25.0)){
      new_trail_range = ((dist_to_enemy - 15.0) / 10.0 ) * (max_trail_range - min_trail_range) + min_trail_range;
    } else if (dist_to_enemy <= 15.0)
      new_trail_range = min_trail_range;;

    
    // check if enemy speed is small
    double enemy_speed = 0.0;
    if (m_node_rec_map[m_name_of_the_opposing_player_who_grabbed_our_flag].isSetSpeed()){
      enemy_speed = m_node_rec_map[m_name_of_the_opposing_player_who_grabbed_our_flag].getSpeed();
      if (enemy_speed < 0.75)
	new_trail_range = 0.0;
    }

    // check if enemy is driving towards us.
    double angle_from_enemy_to_us = relAng(enemy_x, enemy_y, m_nav_x, m_nav_y);
    double enemy_hdg =m_node_rec_map[m_name_of_the_opposing_player_who_grabbed_our_flag].getHeading();

    double hdg_misalignment = angle_from_enemy_to_us - enemy_hdg;
    if (hdg_misalignment < 0)
      hdg_misalignment *= -1.0;

    if (hdg_misalignment < 45.0)
      new_trail_range = min_trail_range;

    // check if the enemy is close, but moving perpendicular
    double heading_diff = angleDiff(m_nav_hdg, enemy_hdg);
    bool moving_perpendicular = ((30.0 < heading_diff) && ( heading_diff < 120.0));
    moving_perpendicular = moving_perpendicular && (enemy_speed > 0.75);

    if (( dist_to_enemy < 25.0) && moving_perpendicular) {
      new_trail_range = max_trail_range; 
    }
      

    // check if this is outside the zone, if so then clip it
    double abs_trail_x, abs_trail_y;
    XYPoint trail_point = projectPoint(enemy_hdg, new_trail_range, enemy_x, enemy_y);

    if (!m_home_zone_poly.contains(trail_point)){
      XYPolygon home_zone_poly_shrunk = m_home_zone_poly;
      home_zone_poly_shrunk.grow_by_amt(-5.0);
      trail_point = home_zone_poly_shrunk.closest_point_on_poly(trail_point); 
      new_trail_range = distPointToPoint(trail_point.get_vx(), trail_point.get_vy(), enemy_x, enemy_y); 
    }

    double filt_trail_range = 0.5 * (new_trail_range + m_last_grabber_intercept_trail_range); 
    Notify("INTERCEPT_FLAG_GRABBER_UPDATES","trail_range=" + doubleToString(new_trail_range));
    m_last_grabber_intercept_trail_range = new_trail_range;
    
  }

  // Is the flag burglar still in our zone?
  bool still_in_zone = (m_names_of_enemy_agents_in_our_zone.count(m_name_of_the_opposing_player_who_grabbed_our_flag) > 0);
  if(m_they_grabbed_our_flag && still_in_zone)
    Notify("BURGLAR_IN_ZONE","true");
  else
    Notify("BURGLAR_IN_ZONE","false");



  
  // Allocate active contacts
  if (m_curr_option != "defend") 
    return;  // don't bother
  
  buildCostMap(); 
  
  // Find best to pursue
  std::string best_intruder_to_persecute = determineBestIntruderToPursue();
  
  if (best_intruder_to_persecute == ""){
    best_intruder_to_persecute =  getContactClosestToFlag();
  }

  Notify("ENEMY_IN_ZONE_INTERCEPT_ONES_UPDATES","contact=" + best_intruder_to_persecute);
  
  // adjust trail dist if needed
  double enemy_speed = 1.5;
  double new_gen_trail_range = 9.0; // the default
  if (m_node_rec_map[best_intruder_to_persecute].isSetSpeed()){
    enemy_speed = m_node_rec_map[best_intruder_to_persecute].getSpeed();
  }
  if (enemy_speed < 0.75)
    new_gen_trail_range = 0.0;

  Notify("ENEMY_IN_ZONE_INTERCEPT_ONES_UPDATES","trail_range=" + doubleToString(new_gen_trail_range));


  
  // update loitering
  unsigned int number_defending = m_pop_state_map["defend"].size();
  if (m_curr_option == "defend")
    number_defending += 1;

  XYPoint near_adj_loiter_pt = m_near_loiter_pt;
  XYPoint far_adj_loiter_pt  = m_far_loiter_pt;

  XYPoint delta_shift = projectPoint(m_loiter_offset_angle + 90.0, m_loiter_offset, 0.0, 0.0);
  near_adj_loiter_pt.shift_x(delta_shift.get_vx());
  near_adj_loiter_pt.shift_y(delta_shift.get_vy());
  
  delta_shift = projectPoint(m_loiter_offset_angle - 90.0, m_loiter_offset, 0.0, 0.0);
  far_adj_loiter_pt.shift_x(delta_shift.get_vx());
  far_adj_loiter_pt.shift_y(delta_shift.get_vy());
  
  
  if (number_defending == 1){
    // just get the near
    Notify("DEFEND_FWD_UPDATES", "station_pt = " + m_near_loiter_pt.get_spec()); 
  } else if (number_defending >=2){
    // Am I the closest?
    std::string closest_to_near_loiter = getTeammateClosestToNearLoiter();
    if (closest_to_near_loiter == m_vname){
      Notify("DEFEND_FWD_UPDATES", "station_pt = " + near_adj_loiter_pt.get_spec());
    } else {
      Notify("DEFEND_FWD_UPDATES", "station_pt = " + far_adj_loiter_pt.get_spec());
    }

  }
  
  
  return;
}

//----------------------------------------------------------
// Procedure: determineBestIntruderToPursue()

std::string AqctNodeCtrl::determineBestIntruderToPursue()
{
 if (m_curr_option != "defend") 
   return("error"); 
  
  double lowest_population_cost = std::numeric_limits<double>::max();
  std::string best_intruder_to_persecute = ""; 
  
  std::map<std::string, double>::iterator it_lv1;
  std::map<std::string, double>::iterator it_lv2;
  std::map<std::string, double>::iterator it_lv3;

  std::string intruder_level_1 = "";
  std::string intruder_level_2 = "";
  std::string intruder_level_3 = "";

  // build a static vector of agents. 
  std::vector<std::string> agent_vec;
  agent_vec.push_back(m_vname);
  std::set<std::string>::iterator it_vec;
  for (it_vec = m_pop_state_map["defend"].begin(); it_vec != m_pop_state_map["defend"].end(); it_vec++){
    agent_vec.push_back(*it_vec); 
  }
  
  double my_cost = 0.0;
  double possible_teammate_2_cost = 0.0;
  double possible_teammate_3_cost = 0.0;

  double teammate_3_cost = 0;
  double teammates_2_and_3_best_cost = 0.0;

  double agent_lv2_lowest_cost = std::numeric_limits<double>::max();
  
  unsigned int number_defending = agent_vec.size();
  unsigned int number_attackers = m_names_of_enemy_agents_in_our_zone.size();
  
  std::cout << "Defense()  starting search" << std::endl;
  
  // m_vname == agent_vec[0]
  for (it_lv1 = m_cost_map[m_vname].begin(); it_lv1 != m_cost_map[m_vname].end(); it_lv1++){
    std::cout << " -Level1" << std::endl;
    
    // what is my cost to pursue this agent?
    intruder_level_1 = it_lv1->first;
    my_cost = it_lv1->second;
    
    std::cout << " *intruder_level_1 = " << intruder_level_1 << std::endl;
    std::cout << " *my_cost =" << doubleToString(my_cost) << std::endl;
    
    teammates_2_and_3_best_cost = 0.0;
    std::cout << " *teammates_2_and_3_best_cost =" << doubleToString( teammates_2_and_3_best_cost) << std::endl;
    
    if (number_defending >= 2){

      if (number_attackers >=2){
	agent_lv2_lowest_cost = std::numeric_limits<double>::max();
      } else {
	agent_lv2_lowest_cost = 0.0;
      }
      std::cout << " **agent_lv2_lowest_cost" << doubleToString(agent_lv2_lowest_cost) << std::endl;
      
      // what is the minimum cost to intercept the other (1-2) agents?
      for (it_lv2 = m_cost_map[agent_vec[1]].begin(); it_lv2 != m_cost_map[agent_vec[1]].end(); it_lv2++){
	std::cout << " --Level2" << std::endl;
	
	// what is teammate 2's cost to pursue this agent?
	intruder_level_2 = it_lv2->first;
	possible_teammate_2_cost = it_lv2->second;

		
	std::cout << " **intruder_level_2 = " << intruder_level_2 << std::endl;
	std::cout << " **possible_teammate_2_cost = " << doubleToString(possible_teammate_2_cost) << std::endl;

	// is this the same agent I am considering?
	if (intruder_level_2 == intruder_level_1)
	  continue;

	// add the cost for the final intruder if there are three intruders
	// otherwise the marginal cost of the third agents decision
	// is zero
	
	teammate_3_cost = 0;

	std::cout << " **teammate_3_cost = " << doubleToString(teammate_3_cost) << std::endl;
	
	if (number_defending == 3){

	  for (it_lv3 = m_cost_map[agent_vec[2]].begin(); it_lv3 != m_cost_map[agent_vec[2]].end(); it_lv3++){
	    std::cout << " ---Level3" << std::endl;
	    
	    // what is teammate 3's cost to pursue this agent?
	    intruder_level_3 = it_lv3->first;
	    possible_teammate_3_cost = it_lv3->second;
	    
	    std::cout << " ***intruder_level_3 = " << intruder_level_3 << std::endl;
	    std::cout << " ***possible_teammate_3_cost = " << doubleToString(possible_teammate_3_cost) << std::endl;
	    
	    if (intruder_level_3 == intruder_level_1)
	      continue;
	    if (intruder_level_3 == intruder_level_2)
	      continue;
	    teammate_3_cost = possible_teammate_3_cost;
	    
	    std::cout << " **teammate_3_cost = " << doubleToString(teammate_3_cost) << std::endl;
	    
	  }  
	}  // by now we have the third agent's cost for this choice at level 2
	
	double total_additional_cost = possible_teammate_2_cost + teammate_3_cost;
	std::cout << " **total_additional_cost = " << doubleToString(total_additional_cost) << std::endl;

	if (total_additional_cost < agent_lv2_lowest_cost){
	  agent_lv2_lowest_cost = total_additional_cost;
	  std::cout << " ** Resetting agent_lv2_lowest_cost" << std::endl;
	}

	std::cout << " **agent_lv2_lowest_cost = " << doubleToString(agent_lv2_lowest_cost) << std::endl;
      }
      // by now agent_lv2_lowest_cost is the lowest posible combo for the remaining two
      // agents and intruders.
      
      teammates_2_and_3_best_cost = agent_lv2_lowest_cost;
      std::cout << " **teammates_2_and_3_best_cost =" << doubleToString( teammates_2_and_3_best_cost) << std::endl;
    }

    
    double population_cost_with_this_option = my_cost + teammates_2_and_3_best_cost;

    std::cout << " *population_cost_with_this_option =" << doubleToString( population_cost_with_this_option) << std::endl;
    if (population_cost_with_this_option < lowest_population_cost){
      std::cout << " *found new best cost, resetting" << std::endl;
      lowest_population_cost = population_cost_with_this_option;
      best_intruder_to_persecute = intruder_level_1;
      std::cout << " *lowest_population_cost =" << doubleToString( lowest_population_cost) << std::endl;
      std::cout << " *best_intruder_to_persecute =" << best_intruder_to_persecute << std::endl;
      
    }
  }
  
  std::cout << " --FINAL -- best_intruder_to_persecute =" << best_intruder_to_persecute << std::endl;
  // by now we have the best intruder

  return(best_intruder_to_persecute); 
}
  

//-------------------------------------------
// Procedure: calcIfClosest(contact_name)
//            returns true if I am the closest of everyone
//            on my team to the contact name
bool AqctNodeCtrl::calcIfClosestTo(std::string contact_name){

  // check if contact is in map
  if (m_node_rec_map.count(contact_name) == 0)
    return false;

  double contact_x = m_node_rec_map[contact_name].getX();
  double contact_y = m_node_rec_map[contact_name].getY();
  
  double dist_from_ownship = distPointToPoint(m_nav_x, m_nav_y, contact_x, contact_y);

  bool i_am_closest = true;

  double x, y, dist;
  std::set<std::string>::iterator it;
  for (it = m_own_team_names.begin(); it != m_own_team_names.end(); it++){
    if (m_node_rec_map.count(*it) == 0)
      continue;
    
    x = m_node_rec_map[*it].getX();
    y = m_node_rec_map[*it].getY();
    dist = distPointToPoint(x, y, contact_x, contact_y);
    if (dist < dist_from_ownship){
      i_am_closest = false;
      break;
    }
  }
  return(i_am_closest); 
}

//-------------------------------------------
// Procedure:getContactClosestToFlag()
std::string AqctNodeCtrl::getContactClosestToFlag(){
  
  double smallest_dist = std::numeric_limits<double>::max();
  double dist;
  std::string closest_contact_to_flag = "";
  
  std::set<std::string>::iterator it;
  for (it = m_names_of_enemy_agents_in_our_zone.begin(); it !=m_names_of_enemy_agents_in_our_zone.end(); it++){

    if (m_node_rec_map.count(*it) == 0)
      continue;
    double contact_x, contact_y;
    contact_x = m_node_rec_map[*it].getX();
    contact_y = m_node_rec_map[*it].getY();
    std::string contact_name = m_node_rec_map[*it].getName();
    
    
    dist = distPointToPoint(contact_x, contact_y, m_own_flag_pt.get_vx(), m_own_flag_pt.get_vy());

    if(dist < smallest_dist){
      smallest_dist = dist;
      closest_contact_to_flag = contact_name; 
    }
  }

  return(closest_contact_to_flag);
}


//-------------------------------------------
// Procedure:getTeammateClosestToNearLoiter()
std::string AqctNodeCtrl::getTeammateClosestToNearLoiter(){

  
  double smallest_dist = std::numeric_limits<double>::max();
  double dist;
  std::string closest_contact_to_near_loiter = "";

  if (m_curr_option == "defend"){
     smallest_dist = distPointToPoint(m_near_loiter_pt.get_vx(), m_near_loiter_pt.get_vy(), m_nav_x, m_nav_y);
     closest_contact_to_near_loiter = m_vname; 
  }
  
  // can anyone beat this?
  if ( m_pop_state_map.count("defend") == 0)
    return (closest_contact_to_near_loiter); 
  
  std::set<std::string>::iterator it;
  for (it = m_pop_state_map["defend"].begin(); it !=m_pop_state_map["defend"].end(); it++){

    if (m_node_rec_map.count(*it) == 0)
      continue;
    double contact_x, contact_y;
    contact_x = m_node_rec_map[*it].getX();
    contact_y = m_node_rec_map[*it].getY();
    std::string contact_name = m_node_rec_map[*it].getName();
    
    dist = distPointToPoint(m_near_loiter_pt.get_vx(), m_near_loiter_pt.get_vy(), contact_x, contact_y);

    if(dist < smallest_dist){
      smallest_dist = dist;
      closest_contact_to_near_loiter = contact_name; 
    }
  }

  return(closest_contact_to_near_loiter);
}



//-------------------------------------------
// Procedure: getCostToIntercept
//            can be any ownship and any contact

double AqctNodeCtrl::getCostToIntercept(std::string teammate, std::string contact){

  // sanity check if contacts are in map
  if (m_node_rec_map.count(teammate) == 0)
    return 0.0;
  if (m_node_rec_map.count(contact) == 0)
    return 0.0;

  // get info
  double friendly_x, friendly_y, friendly_hdg;
  friendly_x = m_node_rec_map[teammate].getX();
  friendly_y = m_node_rec_map[teammate].getY();
  friendly_hdg = m_node_rec_map[teammate].getHeading();

  double contact_x, contact_y, contact_hdg;
  contact_x = m_node_rec_map[contact].getX();
  contact_y = m_node_rec_map[contact].getY();
  contact_hdg = m_node_rec_map[contact].getHeading();

  // project contact position into the future
  double lead = 0.0;
  bool contact_speed_avail = m_node_rec_map[contact].isSetSpeed();
  if (m_use_dynamic_speed_based_lead_dist && contact_speed_avail){
    lead = m_node_rec_map[contact].getSpeed() * m_intercept_speed_extrap;
    if (lead > 50.0)
      lead = 50;
  } else {
    lead = m_intercept_static_lead_dist;
  }
  
  XYPoint future_point = projectPoint(contact_hdg, lead, contact_x, contact_y);

  // Compute dist
  double dist = distPointToPoint(friendly_x, friendly_y, future_point.get_vx(), future_point.get_vy());

  // Add heading penalty
  double heading_diff = angleDiff(friendly_hdg, contact_hdg);
  if (heading_diff < 0)
    heading_diff *= -1.0;
  
  dist += heading_diff * m_intercept_heading_penalty;

  return(dist); 
}




//-------------------------------------------
// Procedure: buildCostMap()
//            builds out m_cost_map

void AqctNodeCtrl::buildCostMap(){

  m_cost_map.clear();
  std::set<std::string>::iterator it; // reused 
  double cost; 

  std::map<std::string, double> agents_costs;
  
  if (m_curr_option == "defend"){
    for (it = m_names_of_enemy_agents_in_our_zone.begin(); it !=m_names_of_enemy_agents_in_our_zone.end(); it++){
      // get cost to intercept this enemy
      cost = getCostToIntercept(m_vname, *it);
      agents_costs[*it] = cost;
    }

    // add to the larer map
    m_cost_map[m_vname] = agents_costs; 
  }
  
  // now add the others
  std::set<std::string>::iterator it2;
  for (it2 = m_pop_state_map["defend"].begin(); it2 != m_pop_state_map["defend"].end(); it2++){
    agents_costs.clear();
    
    for (it = m_names_of_enemy_agents_in_our_zone.begin(); it !=m_names_of_enemy_agents_in_our_zone.end(); it++){
      // get cost to intercept this enemy
      cost = getCostToIntercept(*it2, *it);
      agents_costs[*it] = cost;
    }

    // add to the larer map
    m_cost_map[*it2] = agents_costs; 
  }

  std::cout << "Made cost map, it is:" << std::endl;
  std::map<std::string, std::map<std::string, double>>::iterator it3;
  for (it3 = m_cost_map.begin(); it3 != m_cost_map.end(); it3++){
    std::cout << " @" << it3->first << std::endl;
    std::map<std::string, double>::iterator it4;
    for (it4 = it3->second.begin(); it4 != it3->second.end(); it4++){
      std::cout << "   ->" << it4->first << ", " << doubleToString(it4->second) << std::endl;
    }
  }
  return;
}





/*
//----------------------------------------------------------
// Procedure: buildInterceptMissionTask(name)
std::string AqctNodeCtrl::buildInterceptMissionTask(std::string target_name)
{
    // assemble task desc
    // It should look like:
    //  MISSION_TASK_ALL = type=waypoint,id=wpt$[BIX],waypt_x=$[XPOS],waypt_y=$[YPOS],exempt=rex
    std::string task_desc = "type=convoy,";
    task_desc += "id=convoy" + to_string(m_number_of_tasks_sent) + ",";
    task_desc += "contact=" + target_name + ",";
    task_desc += "task_time=" + doubleToString(MOOSTime()) + ",";

    // who is exempt?
    // the entire enemy team is exempt
    task_desc += "exempt=";
    task_desc += stringSetToString(m_opposing_team_names, ':');
    
    // everyone attacking is exempt
    if (m_pop_state_map.count("attack") > 0){
      if (m_pop_state_map["attack"].size() > 0) {
	std::set<std::string>::iterator it2;
	for (it2 = m_pop_state_map["attack"].begin(); it2 != m_pop_state_map["attack"].end(); it2++){
	  task_desc += ":" + *it2; 
	}

      }
    }
    // if I am attacking, then I'm also exempt,
    if (m_curr_option == "attack")
      task_desc += ":" + m_vname;
    
    return(task_desc); 
}

*/
//----------------------------------------------------------
// Procedure: calculateInputs(time)

void AqctNodeCtrl::publishInputs()
{

  std::map<std::string, double>::iterator it;
  for (it = m_opinion_inputs.begin(); it!= m_opinion_inputs.end(); it++) {
    Notify(it->first, it->second); 
  }
  
  return; 
}


//------------------------------------------------------------
// Procedure: buildReport()

bool AqctNodeCtrl::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "Team Color: " << m_team_color << ",  Size: " << uintToString(m_team_size) << endl;

  m_msgs << "Teammates: ";
  std::set<std::string>::iterator it;
  std::string line; 
  for (it = m_own_team_names.begin(); it != m_own_team_names.end(); it++){
    line += *it + ",";
  }
  if (m_own_team_names.size() > 0) {
    // at least one agent is in the set, remove the last comma
    line = line.substr(0, line.size()-1);
  }
  m_msgs << line << endl;

  m_msgs << "Opposing Team: ";
  std::string other_line; 
  for (it =  m_opposing_team_names.begin(); it !=  m_opposing_team_names.end(); it++){
    other_line += *it + ",";
  }
  if ( m_opposing_team_names.size() > 0) {
    // at least one agent is in the set, remove the last comma
    other_line = other_line.substr(0, other_line.size()-1);
  }
  m_msgs << other_line << endl;

  m_msgs << "                                            " << endl;
  m_msgs << "============================================" << endl;
  if (m_we_grabbed_their_flag){
    m_msgs << " * !We grabbed their flag! Agent " << m_name_of_our_teammate_who_grabbed_their_flag << " has it! " << endl;
    m_msgs << "  -> Enemy agent " << m_name_of_closest_enemy_to_player_with_flag << " is " << doubleToString(m_dist_of_closest_enemy_to_player_with_flag,2) << "m away" << endl;
    m_msgs << "  -> Best trail range = " << doubleToStringX(m_best_trail_dist,2) << ", angle = " << doubleToStringX(m_best_trail_angle,2) << endl;
  } else {
    m_msgs << " * We do not have their flag                 " << endl;
  }

  if (m_they_grabbed_our_flag){
    m_msgs << " * !They grabbed our flag! Agent " << m_name_of_the_opposing_player_who_grabbed_our_flag << " has it! " << endl;
  } else {
    m_msgs << " * They do not have our flag                 " << endl;
  }

   m_msgs << "                                            " << endl;
   m_msgs << "============================================" << endl;
    
   m_msgs << " * Home Zone Poly Spec: " << m_home_zone_poly.get_spec() << endl;
   m_msgs << " * Flag Point Spec:     " << m_own_flag_pt.get_spec() << endl;
   m_msgs << "============================================" << endl;

   m_msgs << "                                            " << endl;

   std::string intruder_line = stringSetToString(m_names_of_enemy_agents_in_our_zone);
   std::string tagged_line = stringSetToString(m_tagged_vehicles);
   
   ACTable actab(4);
   actab << "Dist to own flag | # Agents incoming | Names in Zone | Tagged ";
   actab.addHeaderLines();
   actab << doubleToStringX(m_dist_to_own_flag,2) << doubleToStringX(m_agents_in_our_zone,2) << intruder_line << tagged_line;
   m_msgs << actab.getFormattedString();
   m_msgs << endl;
   
   m_msgs << "                                            " << endl;
   m_msgs << "                                            " << endl;
   m_msgs << "=  Input Values  ==========================================" << endl;
   m_msgs << "===========================================================" << endl;
   std::map<std::string, double>::iterator it2;
   for (it2 = m_opinion_inputs.begin(); it2!= m_opinion_inputs.end(); it2++) {
     m_msgs << "  " << it2->first << ": " <<  doubleToStringX(it2->second, 4) << endl; 
   }
   m_msgs << "                                            " << endl;
   m_msgs << "=  Node Reports   ==========================================" << endl;
   m_msgs << "===========================================================" << endl;
   std::map<std::string, NodeRecord>::iterator it3;
   for (it3 = m_node_rec_map.begin(); it3 != m_node_rec_map.end(); it3++){
     m_msgs << " <- " << it3->second.getSpec() << endl;
   }
   
   return(true);
}




//-----------------------------------------------------
// Handle Node Report
//
bool AqctNodeCtrl::handleNodeReport(std::string msg)
{
 
  // process incoming node record 
  NodeRecord newNodeRecord;
  newNodeRecord = string2NodeRecord(msg, true);
  if (!newNodeRecord.valid())
    return(false);

  // Only keep contacts that are alive:
  if ( m_contacts.count(newNodeRecord.getName()) || (newNodeRecord.getName() == m_vname) )
    m_node_rec_map[newNodeRecord.getName()] = newNodeRecord;
  
  return(true);
 }



//-----------------------------------------------------
// Handle Tagged List Report
//
bool AqctNodeCtrl::handleTaggedList(std::string msg)
{
  // parse message
  std::set<std::string> new_tagged_set;
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i=0; i<svector.size(); i++) {
    new_tagged_set.insert(svector[i]);
  }
  m_tagged_vehicles = new_tagged_set;
  return(true);
}

//-----------------------------------------------------
// Handle Contact List Report
//
bool AqctNodeCtrl::handleContactsList(std::string msg)
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


// --------------------------------------------------------
// Handle population state updates
//
// builds a set that contains the agents participating in each choice
//                    Key = option name
//                    Val = set of names of agents for that option.
bool AqctNodeCtrl::handlePopStateList(std::string key, std::string msg){

  // Make a new set to erase the old set.
  std::set<std::string> new_active_set;
  // parse message
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i=0; i<svector.size(); i++) {
    new_active_set.insert(svector[i]);
  }

  // Some sets could be empty!

  // remove the "_LIST" at the end
  std::string option_name = key.substr(0, key.length()-5);
  option_name = tolower(option_name);

  m_pop_state_map[option_name] = new_active_set; 


  return(true); 
}


// --------------------------------------------------------
//  Handle flag summary message
//
//  Really we only care about who has the flag
//  and we use the RED_FLAG_GRABBED and BLUE_FLAG_GRABBED
//  for status
bool AqctNodeCtrl::handleFlagSummary(std::string msg){


  // does this msg contain "owner=blue_x"?
  std::string common_preamble = "owner=";
  
  std::set<std::string>::iterator it;   //  reused

  // check for names of our teammates first
  for (it = m_own_team_names.begin(); it != m_own_team_names.end(); it++){
    if (strContains(msg, common_preamble + *it) ) {
      m_name_of_our_teammate_who_grabbed_their_flag = *it;
      break;
    }
  }

  // then check for names of members of the opposing team
  for (it = m_opposing_team_names.begin(); it != m_opposing_team_names.end(); it++){
    if (strContains(msg, common_preamble + *it) ) {
      m_name_of_the_opposing_player_who_grabbed_our_flag = *it;
      Notify("INTERCEPT_FLAG_GRABBER_UPDATES","contact=" + m_name_of_the_opposing_player_who_grabbed_our_flag);
      break;
    }
  }
  
  return(true); 
}


/*
// --------------------------------------------------------
//  Handle Intruder Tasked message
//
//   Generate a new random time to retask this intruder
//   each node will have a different time, so this will help
//   with conflicts
bool AqctNodeCtrl::handleIntruderTasked(std::string msg){

  std::string name = tolower(msg);
  if (name == "")
    return(false);

  double random_interval = randomDouble(m_min_reallocate_interval, m_max_reallocate_interval);
  m_time_to_reassign_intruder_map[name] = MOOSTime() + random_interval;

  return(true);
}
*/
