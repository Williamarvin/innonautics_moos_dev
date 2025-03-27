/************************************************************/
/*    NAME: Filip Stromstad                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DynamicTrafficLight.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <algorithm>
#include "MBUtils.h"
#include "ACTable.h"
#include "DynamicTrafficLight.h"
#include "NodeRecord.h"
#include "NodeMessage.h" // In the lib_ufield library
#include "XYFormatUtilsSegl.h"
#include "XYFormatUtilsPoly.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

DynamicTrafficLight::DynamicTrafficLight()
{
  m_name_os = "";
  m_precision = 1;
  m_default_speed = 1.0;

  m_max_safety_dist = 4;
  m_min_safety_dist = 2;
  m_max_horizon_meters = 10;
  m_min_horizon_meters = 2;
  m_use_dynamic_speed = false;
  m_show_visualization = false;
  m_synchronize = false;
  m_turn_in_place = false;

  m_trajectory_map = {};
  m_speed_map = {};
  m_block_matrix = {};
  m_temp_block_dist_matrix = {};
  m_temp_block_limit = 5;
  
  m_handshakes = {};
  m_nodes_in_proximity = {};
  m_proximity_range = 15;
  m_os_trajectory_number = 0;
  m_known_traj_numbers = {};

  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_heading = 0;
  m_compass_heading = 0;
  m_use_compass_heading = false;
  m_nav_speed = 0;
  
  m_potential_permablock_timer = -1;
  m_potential_deadlock_timer = -1;
  m_deadlock_timer = -1;

  m_mode = "decluster"; //decluster, simultaneous, sequential
  m_given_control = true; //For use in sequential mode

  m_reverse_thrust = -30;
}

//---------------------------------------------------------
// Destructor

DynamicTrafficLight::~DynamicTrafficLight()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool DynamicTrafficLight::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval  = msg.GetString(); 
    double mtime = msg.GetTime();
    double dval  = msg.GetDouble();

    if(key == "TRAJECTORY_MSG_LOCAL") {
      XYSegList temp_seglist = string2SegList(sval);
      string label = temp_seglist.get_label(); //label set previously to "osname_dubin_#oftrajectories"
      int traj_number = stoi(label.substr(label.find_last_of("_") + 1));

      //Higher trajectory number --> newer trajectory
      if (traj_number > m_os_trajectory_number) {
        m_os_trajectory_number = traj_number;
        m_handshakes.clear();
      }

      m_trajectory_map[m_name_os] = temp_seglist;

      NodeMessage node_msg;
      node_msg.setSourceNode(m_name_os);
      node_msg.setDestNode("all");
      node_msg.setVarName("TRAJECTORY_MSG");
      node_msg.setStringVal(temp_seglist.get_spec());
      Notify("NODE_MESSAGE_LOCAL", node_msg.getSpec());
     } 
     else if(key == "TRAJECTORY_MSG") {
       XYSegList temp_seglist = string2SegList(sval);
       string traj_label = temp_seglist.get_label(); //label on the form "osname_dubin_#oftrajectories"
       string sender_name = traj_label.substr(0, traj_label.find_first_of("_"));
       string traj_number = traj_label.substr(traj_label.find_last_of("_") + 1);

       m_trajectory_map[sender_name] = temp_seglist;
       m_known_traj_numbers[sender_name] = traj_number;

      //  Check if sender is in proximity, position is the first point in the trajectory
       XYPoint position = temp_seglist.get_first_point();
       double dist = sqrt(pow(position.x()-m_nav_x, 2) + pow(position.y()-m_nav_y, 2));
       if (dist < m_proximity_range && find(m_nodes_in_proximity.begin(), m_nodes_in_proximity.end(), sender_name) == m_nodes_in_proximity.end()) {
         m_nodes_in_proximity.push_back(sender_name);
       } else if (dist >= m_proximity_range && find(m_nodes_in_proximity.begin(), m_nodes_in_proximity.end(), sender_name) != m_nodes_in_proximity.end()) {
         m_nodes_in_proximity.erase(remove(m_nodes_in_proximity.begin(), m_nodes_in_proximity.end(), sender_name), m_nodes_in_proximity.end());
       }
     } 
     else if (key == "TRAJECTORY_HANDSHAKE"){
        string sender_name = parseString(sval, ',')[0];
        int traj_number = stoi(parseString(sval, ',')[1]);
        bool sender_in_handshakes = find(m_handshakes.begin(), m_handshakes.end(), sender_name) != m_handshakes.end();
        bool traj_number_most_recent = (traj_number >= m_os_trajectory_number); //should not be possible to be greater than

        if (!sender_in_handshakes && traj_number_most_recent) {
          m_handshakes.push_back(sender_name);
        }
     }
     else if(key == "SPEED_MSG") {
        vector<string> parts = parseString(sval, ',');
        m_speed_map[parts[0]] = stod(parts[1]);
     }
     else if(key == "NAV_X") {
       m_nav_x = dval;
     } 
     else if(key == "NAV_Y") {
       m_nav_y = dval;
     }
     else if (key == "NAV_HEADING") {
       m_nav_heading = dval;
     }
     else if (key == "COMPASS_HEADING_RAW") {
       m_compass_heading = dval;       
     }
     else if(key == "NAV_SPEED") {
       m_nav_speed = dval;
       m_speed_map[m_name_os] = dval;

       NodeMessage node_msg;
       node_msg.setSourceNode(m_name_os);
       node_msg.setDestNode("all");
       node_msg.setVarName("SPEED_MSG");
       node_msg.setStringVal(m_name_os + "," + to_string(dval));
       Notify("NODE_MESSAGE_LOCAL", node_msg.getSpec());
     }
     else if(key == "DEMUSTER_CONFIG"){
        if (sval == "synch_toggle"){
          m_synchronize = !m_synchronize;
        } 
        else if (sval == "synch_true"){
          m_synchronize = true;
        } 
        else if (sval == "synch_false"){
          m_synchronize = false;
        }
        else if (sval == "turn_in_place_true"){
          m_turn_in_place = true;
        } 
        else if (sval == "turn_in_place_false"){
          m_turn_in_place = false;
        }
        else if (sval == "safety_distance_increase"){
          m_max_safety_dist += 1;
        } 
        else if (sval == "safety_distance_decrease"){
          m_max_safety_dist -= 1;
        }
        else if (sval.find("safety_distance_set=") == 0){
          double set_value = stod(sval.substr(20));
          m_max_safety_dist = set_value;
          m_min_safety_dist = set_value;
        }
        else if (sval == "safety_horizon_increase"){
          m_max_horizon_meters += 1;
        } 
        else if (sval == "safety_horizon_decrease"){
          m_max_horizon_meters -= 1;
        }
        else if (sval == "speed_increase"){
          m_default_speed += 0.1;
        } 
        else if (sval == "speed_decrease"){
          m_default_speed -= 0.1;
        }
     }
     else if (key == "YOU_HAVE_CONTROL"){
       if (m_mode == "sequential"){
         m_given_control = (sval == "true");
       }
     }
     else if(key != "APPCAST_REQ"){ // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
     }
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool DynamicTrafficLight::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool DynamicTrafficLight::Iterate()
{
  AppCastingMOOSApp::Iterate();

  //Send out handshakes
  // only send out handshakes every x seconds
  if (MOOSTime() - m_last_handshake_time > 2){
    m_last_handshake_time = MOOSTime();

    for (auto const& node : m_known_traj_numbers) { //TODO: could reduce the frequency of this
        // find distance to the node
        XYPoint position = m_trajectory_map[node.first].get_first_point();
        double dist = sqrt(pow(position.x()-m_nav_x, 2) + pow(position.y()-m_nav_y, 2));
        
        if (dist > m_proximity_range * 1.2){
          continue;
        }


        XYSegList temp_seglist = m_trajectory_map[m_name_os];
        NodeMessage traj_msg;
        traj_msg.setSourceNode(m_name_os);
        traj_msg.setDestNode(node.first);
        traj_msg.setVarName("TRAJECTORY_MSG");
        traj_msg.setStringVal(temp_seglist.get_spec());
        Notify("NODE_MESSAGE_LOCAL", traj_msg.getSpec());

        NodeMessage handshake_msg;
        handshake_msg.setSourceNode(m_name_os);
        handshake_msg.setDestNode(node.first);
        handshake_msg.setVarName("TRAJECTORY_HANDSHAKE");
        handshake_msg.setStringVal(m_name_os + "," + node.second);
        Notify("NODE_MESSAGE_LOCAL", handshake_msg.getSpec());
    }
  }

  bool break_early = false;
  if (m_trajectory_map.find(m_name_os) == m_trajectory_map.end()){ //If no trajectory, stop
    break_early = true;
  } 
  else if (m_trajectory_map[m_name_os].size() == 0) { //No os trajectory, stop
    break_early = true;
  }
  else if (m_trajectory_map[m_name_os].size() <= 1) {
    //os has arrived at the last node or has no nodes
    break_early = true;
  }
  else if (m_mode == "simultaneous") {
    break_early = true;
  }

  // check if all nodes in proximity have handshook
  for (auto const& node : m_nodes_in_proximity) {
    if (find(m_handshakes.begin(), m_handshakes.end(), node) == m_handshakes.end()) {
      break_early = true;
      break;
    }
  }

  if (break_early){
    Notify("DUBIN_UPDATE", "speed=0");
    if (m_mode == "sequential"){
      Notify("I_WANT_CONTROL", "false");
    }

    if (m_show_visualization){
      string label = "safety_" + m_name_os;
      XYPolygon poly = XYPolygon(0, 0, 10, 10, label);
      poly.set_active(false);
      Notify("VIEW_POLYGON", poly.get_spec());
      XYSegList viz_seglist = XYSegList("traj_" + m_name_os);
      Notify("VIEW_SEGLIST", viz_seglist.get_spec_inactive());
    }

    AppCastingMOOSApp::PostReport();
    return(true);
  }


  // Deadlock handling
  if (m_deadlock_timer != -1){
    if (MOOSTime() - m_deadlock_timer < 10){
      //Deadlocked
      Notify("DEADLOCKED", "true");

      string label = "safety_" + m_name_os;
      // Make a squarepolygon right behind ownship
      double poly_radius = 10;

      double os_heading_rad = 0;
      double vector_angle = 0;

      if (m_use_compass_heading){
        os_heading_rad = (90 - m_compass_heading) * M_PI / 180;
        vector_angle = m_compass_heading;
      } else {
        os_heading_rad = (90 - m_nav_heading) * M_PI / 180;
        vector_angle = m_nav_heading;
      }

      XYPolygon poly = XYPolygon();
      poly.set_label(label);
      poly.add_vertex(m_nav_x, m_nav_y);

      int lim = 8;
      for (int i = -lim; i <= lim; i++){ //Between -lim*10 and lim*10 degrees
        double relative_angle = 10 * i * M_PI / 180;
        double angle_1 = os_heading_rad - M_PI + relative_angle;
        double poly_x_1 = m_nav_x + poly_radius * cos(angle_1);
        double poly_y_1 = m_nav_y + poly_radius * sin(angle_1);
        poly.add_vertex(poly_x_1, poly_y_1);
      }

      if (!poly.is_convex()){
        //Will not be convex if relative angle is >=90 (= beacuse of rounding errors)
        reportRunWarning("Polygon is not convex");
      }

      //Check if any of the other vehicles are in the polygon
      bool blocked = false;
      for (auto const& vehicle : m_trajectory_map) {
        string name = vehicle.first;
        if (name == m_name_os) {
          continue;
        }
        XYSegList seglist = vehicle.second;
        XYPoint position = seglist.get_first_point();
        if (poly.contains(position.x(), position.y())) {
          blocked = true;
          break;
        }
      }


      string viz_color = "";
      if (blocked){
        //Reset the deadlock timer
        if (m_mode == "sequential"){
          Notify("I_WANT_CONTROL", "false");
        }
        Notify("DESIRED_THRUST", 0.0);
        // m_deadlock_timer = MOOSTime();
        viz_color = "red";
      } 
      else {
        if (m_mode == "sequential"){
          Notify("I_WANT_CONTROL", "true");
        }
        if (m_given_control){
          Notify("DESIRED_THRUST", m_reverse_thrust);
        } else {
          Notify("DESIRED_THRUST", 0.0);
        }
        // Notify("DESIRED_THRUST", m_reverse_thrust);
        viz_color = "green";
      }

      if (m_show_visualization){
        // poly.set_edge_color(viz_color);
        // poly.set_vertex_color(viz_color);
        // poly.set_label_color("transparent");
        // Notify("VIEW_POLYGON", poly.get_spec());
      }


    } else  {
      //Remove os trajectory from trajectory map
      m_trajectory_map.erase(m_name_os);
      m_deadlock_timer = -1;
      Notify("MOOS_MANUAL_OVERRIDE", "false");
      Notify("DUBIN_UPDATE", "regenerate_path=true");
    }

    AppCastingMOOSApp::PostReport();
    return(true);
  }

  //TODO:
  // Only check os first
  // If total status is blocked or unblocked, no need to check the rest...
  // If total status is temp_blocked, check all other vehicles to make final decision

  //double for-loop to compare all trajectories to each other
  string horizon_color = "green";

  for (auto const& vehicle_i : m_trajectory_map) { //For all vehicles...
    string name_i = vehicle_i.first;

    // continue if the vehicle is not in proximity and not the os
    if (find(m_nodes_in_proximity.begin(), m_nodes_in_proximity.end(), name_i) == m_nodes_in_proximity.end() && name_i != m_name_os) {
      m_block_matrix[name_i]["total"] = PHYSICALLY_BLOCKED; //Assume physically blocked for handling deadlocks
      // Assume vehicle I is PHYSICALLY_BLOCKED by all vehicles
      for (auto const& vehicle_j : m_trajectory_map) {
        string name_j = vehicle_j.first;
        if (name_i == name_j) {
          m_block_matrix[name_i][name_j] = UNBLOCKED; //Cannot be blocked by itself
          m_temp_block_dist_matrix[name_i][name_j] = -1;
          continue;
        }
        m_block_matrix[name_i][name_j] = PHYSICALLY_BLOCKED;
        m_temp_block_dist_matrix[name_i][name_j] = -1;
      }
      continue;
    }
    

    XYSegList seglist_i = vehicle_i.second;
    double speed_i = m_speed_map[name_i];
    double dynamic_safe_dist_i = m_min_safety_dist + min(pow((speed_i/m_default_speed),2),1.0) * (m_max_safety_dist - m_min_safety_dist);
    double dynamic_horizon_i_meters = m_min_horizon_meters + min(pow((speed_i/m_default_speed),2),1.0) * (m_max_horizon_meters - m_min_horizon_meters);
    int dynamic_horizon_i_points = round(dynamic_horizon_i_meters / m_precision);
    int horizon_i = min(dynamic_horizon_i_points, int(m_trajectory_map[name_i].size()));

    for (auto const& vehicle_j : m_trajectory_map) { //..check block status with all other vehicles
      string name_j = vehicle_j.first;
      if (name_i == name_j) {
        m_block_matrix[name_i][name_j] = UNBLOCKED; //Cannot be blocked by itself
        m_temp_block_dist_matrix[name_i][name_j] = -1;
        continue;
      }

      // continue if the vehicle is not in proximity and not the os
      if (find(m_nodes_in_proximity.begin(), m_nodes_in_proximity.end(), name_j) == m_nodes_in_proximity.end() && name_i == m_name_os) {
        m_block_matrix[name_i][name_j] = UNBLOCKED; //Cannot be blocked by vehicles not in proximity
        m_temp_block_dist_matrix[name_i][name_j] = -1;
        continue;
      }


      XYSegList seglist_j = vehicle_j.second;
      double speed_j = m_speed_map[name_j];
      double dynamic_safe_dist_j = m_min_safety_dist + min(pow((speed_j/m_default_speed),2),1.0)* (m_max_safety_dist - m_min_safety_dist);
      double largest_safe_dist = max(dynamic_safe_dist_i, dynamic_safe_dist_j);
      double dynamic_horizon_j_meters = m_min_horizon_meters + min(pow((speed_j/m_default_speed),2),1.0) * (m_max_horizon_meters - m_min_horizon_meters);
      int dynamic_horizon_j_points = round(dynamic_horizon_j_meters / m_precision);
      int horizon_j = min(dynamic_horizon_j_points, int(m_trajectory_map[name_j].size()));

      //----------1. Check if vehicle i is blocked by vehicle j----------
      bool physically_blocked = false;
      XYPoint position_j = seglist_j.get_first_point();

      // TODO(?): This for loop could technically be merged with the temp blocked check (if j = 0, then phys. blocked)
      double dist_to_intersection = -1;
      for (int i = 0; i < horizon_i; i++) { //For all points in trajectory i
        XYPoint point_i = seglist_i.get_point(i);
        double dist = sqrt(pow(point_i.x()-position_j.x(), 2) + pow(point_i.y()-position_j.y(), 2));
        if (dist < largest_safe_dist) {
          physically_blocked = true;
          dist_to_intersection = i * m_precision;
          break;
        }
      }

      if (physically_blocked) {
        m_block_matrix[name_i][name_j] = PHYSICALLY_BLOCKED;
        // m_temp_block_dist_matrix[name_i][name_j] = -1;
        m_temp_block_dist_matrix[name_i][name_j] = dist_to_intersection;
        if (name_i == m_name_os) {
          horizon_color = "red";
        }
        continue;
      }

      //----------2. Check if vehicle i is temporarily blocked by vehicle j----------
      bool temp_blocked = false;
      for (int i = 0; i < horizon_i; i++) { //For all points in trajectory i
        XYPoint point_i = seglist_i.get_point(i);
        for (int j = 0; j < horizon_j; j++) { //For all points in trajectory j
          XYPoint point_j = seglist_j.get_point(j);
          double dist = sqrt(pow(point_i.x()-point_j.x(), 2) + pow(point_i.y()-point_j.y(), 2));
          if (dist < largest_safe_dist) {
            temp_blocked = true;
            double distance_to_temp_block = i * m_precision;
            m_temp_block_dist_matrix[name_i][name_j] = distance_to_temp_block;
            break;
          }
        }
        if (temp_blocked) {
          break;
        }
      }

      if (temp_blocked) {
          m_block_matrix[name_i][name_j] = TEMP_BLOCKED;
          if (horizon_color != "red" && name_i == m_name_os) { //Phys. blocked has priority
            horizon_color = "orange";
          }
      } else { //vehicle i is not physically or temporarily blocked by vehicle j
        m_block_matrix[name_i][name_j] = UNBLOCKED;
        m_temp_block_dist_matrix[name_i][name_j] = -1;
      }

    }
  }

  //----------3. For all temp blocked, decide on the actual blockage----------
  for (auto const& block_map : m_block_matrix) {
    string current = block_map.first;
    for (auto const& blockage : block_map.second) {
      string other = blockage.first;
      if (other == "total" || other == current) {
        continue;
      }
      if (blockage.second != TEMP_BLOCKED) {
        continue;
      }
      
      BlockStatus current_to_other = UNBLOCKED;
      double current_dist_to_block = m_temp_block_dist_matrix[current][other];
      double other_dist_to_block = m_temp_block_dist_matrix[other][current];
      double decision_dist = current_dist_to_block - other_dist_to_block;
      if (m_block_matrix[other][current] == PHYSICALLY_BLOCKED) { //other one is physically blocked (or previously set to blocked bc of priority)
        //Current one is unblocked by the other one because it is physically blocking it
      } 
      else if (decision_dist < -m_temp_block_limit) { //"I" am more than 5 meters ahead of the other one
        //Current one is unblocked by the other one because of distance
        // horizon_color = "magenta";
      } 
      else if (decision_dist > m_temp_block_limit) { //The other one is more than 5 meters ahead of me
        //Current one is blocked by the other one because of distance
        // horizon_color = "magenta";
        current_to_other = PHYSICALLY_BLOCKED;
      }
      else {
        int decision_var = current.compare(other);
        if (decision_var < 0) {
          //Current one is unblocked by the other one because of priority
        } 
        else {
          //Current one is blocked by the other one because of priority
          current_to_other = PHYSICALLY_BLOCKED;
        }
      }
      m_block_matrix[current][other] = current_to_other;
    }
  }  

  //----------4. Decide on the total blockstatus of each ----------
  //TODO: ONLY NEED TO FIND MY OWN TOTAL STATUS? COULD REMMOVE FROM MATRIX AND JUST HAVE A VARIABLE?
  for (auto const& block_map : m_block_matrix) {
    string name = block_map.first;
    BlockStatus total_status = UNBLOCKED;
    for (auto const& blockage : block_map.second) {
      if (blockage.first == "total") {
        continue;
      }

      BlockStatus status = blockage.second;
      if (status == PHYSICALLY_BLOCKED) {
        total_status = PHYSICALLY_BLOCKED;
        break;
      } else if (status == TEMP_BLOCKED) {
        total_status = TEMP_BLOCKED;
      }
    }
    m_block_matrix[name]["total"] = total_status;
  }  

  double synch_factor = 1;
  // if (m_synchronize) {
  if (m_synchronize && m_nodes_in_proximity.size() > 0) {
    //find the average total pathlength left for each vehicle
    double total_pathlength = 0;
    // for (auto const& vehicle : m_trajectory_map) {
    //   XYSegList seglist = vehicle.second;
    //   total_pathlength += seglist.length();
    // }
    // double average_pathlength = total_pathlength / m_trajectory_map.size();
    
    // Only consider the vehicles in proximity
    for (auto const& vehicle : m_nodes_in_proximity) {
      XYSegList seglist = m_trajectory_map[vehicle];
      total_pathlength += seglist.length();
    }

    double average_pathlength = total_pathlength / m_nodes_in_proximity.size();
    double os_pathlength = m_trajectory_map[m_name_os].length();
    synch_factor =  min(os_pathlength / average_pathlength, 1.4);
    synch_factor =  max(synch_factor, 0.1);
  }

  //----------5. Send speed command to vehicles based on blockstatus----------
  string viz_color = ""; //TODO: MOVE THIS UP SO ITS GLOBAL AND DECIDE IN THE MAIN LOOP. START GREEN, IF TEMP-ORANGE, IF PHYSICALLY-RED
  if (m_block_matrix[m_name_os]["total"] == PHYSICALLY_BLOCKED) {
    // for all the vehicles I am physically blocked by, find the one with the shortest distance to the intersection
    double shortest_dist = -1;
    for (auto const& blockage : m_block_matrix[m_name_os]) {
      string other = blockage.first;
      if (other == "total" || blockage.second != PHYSICALLY_BLOCKED) {
        continue;
      }
      double dist = m_temp_block_dist_matrix[m_name_os][other];
      if (shortest_dist == -1 || dist < shortest_dist) {
        shortest_dist = dist;
      }
    }

    if (shortest_dist == -1){
      // something went wrong
      reportRunWarning("No shortest distance found, even though physically blocked");
    }

    double desired_speed = 0;
    double min_test = 6;
    // double min_test = m_min_horizon_meters;
    double max_test = m_max_horizon_meters;
    // double min_test = 3;
    // double max_test = 10;
    if (shortest_dist > min_test && m_use_dynamic_speed){
      // We know shortest_dist must be between min_horizon and max_horizon
      double horizon_int = max_test - min_test;
      double distance_diff = shortest_dist - min_test;
      // desired_speed = min(m_default_speed * pow(distance_diff/horizon_int, 1), m_default_speed);
      desired_speed = min(m_default_speed * pow(distance_diff/horizon_int, 2), m_default_speed);
      desired_speed = ceil(desired_speed * 10) / 10; // Round desired speed up to nearest 0.1
      // horizon_color = "magenta";
    }

    // Notify("DUBIN_UPDATE", "speed=0");
    // Notify("DUBIN_UPDATE", "speed="+doubleToString(desired_speed));

    if (desired_speed == 0){
      if (m_mode == "sequential"){
        Notify("I_WANT_CONTROL", "false");
      }
      Notify("DUBIN_UPDATE", "speed="+doubleToString(desired_speed));
    } else {
      if (m_mode == "sequential"){
        Notify("I_WANT_CONTROL", "true");
      }
      if (m_given_control){
        Notify("DESIRED_THRUST", "speed="+doubleToString(desired_speed));
      } else {
        Notify("DESIRED_THRUST", 0.0);
      }
    }



    viz_color = "red";
    if (m_turn_in_place){
      if (desired_speed == 0 && m_nav_speed < 0.4){
        Notify("TIP_UPDATE", "point="+m_trajectory_map[m_name_os].get_last_point().get_spec());
        Notify("CHANGE_DRIVE_MODE", "direct");
        Notify("TURN_IN_PLACE", "true");
        Notify("DUBIN_UPDATE", "regenerate_path");
      } else {
        Notify("TURN_IN_PLACE", "false");
        Notify("CHANGE_DRIVE_MODE", "aggro");
      }
    }
  } else {
    // Notify("DUBIN_UPDATE", "speed="+doubleToString(m_default_speed*synch_factor)); //Unblocked
    if (m_mode == "sequential"){
      Notify("I_WANT_CONTROL", "true");
    }
    if (m_given_control){
      Notify("DUBIN_UPDATE", "speed="+doubleToString(m_default_speed*synch_factor)); //Unblocked
    } else {
      Notify("DUBIN_UPDATE", "speed=0");
    }

    viz_color = "green";
    if (m_turn_in_place){
      Notify("TURN_IN_PLACE", "false");
      Notify("CHANGE_DRIVE_MODE", "aggro");
    }
  }
  //Should not be possible to be temp_blocked here

  //Deadlock detection
  bool potential_permablock = false;
  bool potential_deadlock = false;
  if (m_block_matrix[m_name_os]["total"] == PHYSICALLY_BLOCKED) {
    vector<string> visited;
    vector<string> recursive_stack;
    potential_deadlock = depthFirstSearch(m_name_os, m_block_matrix, visited, recursive_stack);
    potential_permablock = isPermablocked(m_name_os, m_block_matrix, m_trajectory_map);
  }

  if (potential_permablock){
    potential_deadlock = false; //Permablocked has priority
    if (m_potential_permablock_timer == -1){
      m_potential_permablock_timer = MOOSTime();
    } 
    else if (MOOSTime() - m_potential_permablock_timer > 10) {
      //Notify shoreside
      Notify("PERMANENTLY_BLOCKED", "true");
      Notify("DUBIN_UPDATE", "speed=0");
      if (m_mode == "sequential"){
        Notify("I_WANT_CONTROL", "false");
      }

      XYSegList output_seglist;
      output_seglist.add_vertex(m_nav_x, m_nav_y);
      output_seglist.set_label(m_name_os + "_dubin_" + to_string(m_os_trajectory_number+1));

      NodeMessage node_msg;
      node_msg.setSourceNode(m_name_os);
      node_msg.setDestNode("all");
      node_msg.setVarName("TRAJECTORY_MSG");
      node_msg.setStringVal(output_seglist.get_spec());
      Notify("NODE_MESSAGE_LOCAL", node_msg.getSpec());
    }
  } else { 
    m_potential_permablock_timer = -1;
  }

  if (potential_deadlock){
    if (m_potential_deadlock_timer == -1){
      m_potential_deadlock_timer = MOOSTime();
    } 
    else if (MOOSTime() - m_potential_deadlock_timer > 10) {
      m_deadlock_timer = MOOSTime();
      Notify("MOOS_MANUAL_OVERRIDE", "true");
    }
  } else {
    m_potential_deadlock_timer = -1;
  }

  
  if (m_show_visualization){
    //Vizualise safety radius
    string label = "safety_" + m_name_os;
    double dynamic_safety_dist = m_min_safety_dist + min(pow((m_nav_speed/m_default_speed),2),1.0) * (m_max_safety_dist - m_min_safety_dist);
    XYPolygon poly = XYPolygon(m_nav_x, m_nav_y, dynamic_safety_dist, 10, label);
    poly.set_edge_color(viz_color);
    poly.set_vertex_color(viz_color);
    poly.set_label_color("invisible");
    Notify("VIEW_POLYGON", poly.get_spec());

    //Visualize the horizon trajectory
    // double dynamic_horizon_os_meters = m_min_horizon_meters + min(m_nav_speed/m_default_speed, 1.0) * (m_max_horizon_meters - m_min_horizon_meters);
    double dynamic_horizon_os_meters = m_min_horizon_meters + min(pow((m_nav_speed/m_default_speed),2),1.0) * (m_max_horizon_meters - m_min_horizon_meters);
    int dynamic_horizon_os_points = round(dynamic_horizon_os_meters / m_precision);
    int horizon = min(dynamic_horizon_os_points, int(m_trajectory_map[m_name_os].size()));
    XYSegList viz_seglist = XYSegList("traj_" + m_name_os);
    for (int i = 1; i < horizon; i++) {
      viz_seglist.add_vertex(m_trajectory_map[m_name_os].get_point(i));
    }

    viz_seglist.set_edge_color(horizon_color);
    viz_seglist.set_edge_size(3);
    viz_seglist.set_label_color("invisible");
    viz_seglist.set_vertex_color("invisible");
    Notify("VIEW_SEGLIST", viz_seglist.get_spec());
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool DynamicTrafficLight::OnStartUp()
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
    if(param == "vname") {
      m_name_os = value;
      handled = true;
    } else if (param == "default_speed") {
      m_default_speed = stod(value);
      handled = true;
    } else if (param == "precision") {
      m_precision = stod(value);
      handled = true;
    } else if (param == "max_safety_distance") {
      m_max_safety_dist = stoi(value);
      handled = true;
    } else if (param == "min_safety_distance") {
      m_min_safety_dist = stoi(value);
      handled = true;
    } else if (param == "max_horizon") {
      m_max_horizon_meters = stoi(value);
      handled = true;
    } else if (param == "min_horizon") {
      m_min_horizon_meters = stoi(value);
      handled = true;
    } else if (param == "show_visualization") {
      m_show_visualization = (value == "true");
      handled = true;
    } else if (param == "use_dynamic_speed") {
      m_use_dynamic_speed = (value == "true");
      handled = true;
    } else if (param == "synchronize") {
      m_synchronize = (value == "true");
      handled = true;
    } else if (param == "reverse_thrust"){
      m_reverse_thrust = stod(value);
      handled = true;
    } else if (param == "use_compass_heading"){
      m_use_compass_heading = (value == "true");
      handled = true;
    } else if (param == "temp_block_limit"){
      m_temp_block_limit = stod(value);
      handled = true;
    } else if (param == "mode"){
      if (value == "decluster" || value == "simultaneous" || value == "sequential" || value == "deploy"){
        m_mode = value;
        if (m_mode == "sequential"){
          m_given_control = false; //Needs to be given control by shoreside
        }
        handled = true;
      }
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void DynamicTrafficLight::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("TRAJECTORY_MSG_LOCAL", 0);
  Register("TRAJECTORY_MSG", 0);
  Register("TRAJECTORY_HANDSHAKE", 0);
  Register("SPEED_MSG", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("COMPASS_HEADING_RAW", 0);
  Register("NAV_SPEED", 0);
  Register("DEMUSTER_CONFIG", 0);
  Register("YOU_HAVE_CONTROL", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool DynamicTrafficLight::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: DynamicTrafficLight.cpp               " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "Configuration parameters for " << m_name_os << ":" << endl;
  m_msgs << "  Default speed: " << m_default_speed << endl;
  m_msgs << "  Actual speed: " << m_nav_speed << endl;
  m_msgs << "  Precision: " << m_precision << endl;
  m_msgs << "  Max safety distance: " << m_max_safety_dist << endl;
  m_msgs << "  Min safety distance: " << m_min_safety_dist << endl;
  double dynamic_safety_dist = m_min_safety_dist + min(pow((m_nav_speed/m_default_speed),2),1.0) * (m_max_safety_dist - m_min_safety_dist);
  m_msgs << "  Dynamic safety distance: " << dynamic_safety_dist << endl;
  m_msgs << "  Max horizon: " << m_max_horizon_meters << endl;
  m_msgs << "  Min horizon: " << m_min_horizon_meters << endl;
  double dynamic_horizon_os_meters = m_min_horizon_meters + min(pow((m_nav_speed/m_default_speed),2),1.0) * (m_max_horizon_meters - m_min_horizon_meters);
  m_msgs << "  Dynamic horizon: " << dynamic_horizon_os_meters << endl;
  m_msgs << "  Temp block limit: " << m_temp_block_limit << endl;
  m_msgs << "  Show visualization: " << m_show_visualization << endl;
  m_msgs << "  Synchronize: " << m_synchronize << endl;
  m_msgs << "  Turn in place: " << m_turn_in_place << endl;
  m_msgs << "  Use dynamic speed: " << m_use_dynamic_speed << endl;
  m_msgs << "  Mode: " << m_mode << endl;

  m_msgs << "============================================" << endl;
  string handshake_nodes = "";
  for (auto const& x : m_handshakes)
  {
    handshake_nodes += x + " ";
  }
  m_msgs << "Handshakes: " << handshake_nodes << endl;
  string proximity_nodes = "";
  for (auto const& x : m_nodes_in_proximity)
  {
    proximity_nodes += x + " ";
  }
  m_msgs << "Nodes in proximity: " << proximity_nodes << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Own Trajectory: " << m_trajectory_map[m_name_os].get_spec() << endl;
  // m_msgs << "Other Trajectories: " << endl;
  // m_msgs << "Other trajectories length: " << m_trajectory_map.size() << endl;
  // for (auto const& x : m_trajectory_map)
  // {
  //   m_msgs << x.first << " --> " << x.second.get_spec() << endl;

  // }

  m_msgs << "============================================" << endl;
  m_msgs << "Block matrix: " << endl;
  for (auto const& x : m_block_matrix)
  {
    if (x.first != m_name_os){
      continue;
    }
    m_msgs << x.first << " --> ";
    for (auto const& y : x.second)
    {
      m_msgs << y.first << ": " << y.second << " ";
    }
    m_msgs << endl;
  }
  m_msgs << "Temp block matrix: " << endl;
  for (auto const& x : m_temp_block_dist_matrix)
  {
    if (x.first != m_name_os){
      continue;
    }
    m_msgs << x.first << " --> ";
    for (auto const& y : x.second)
    {
      m_msgs << y.first << ": " << y.second << " ";
    }
    m_msgs << endl;
  }

  m_msgs << "============================================" << endl;
  m_msgs << "Potential permablock timer: " << m_potential_permablock_timer << endl;
  m_msgs << "Potential deadlock timer: " << m_potential_deadlock_timer << endl;

  m_msgs << "============================================" << endl;
  m_msgs << "Use compass heading: " << m_use_compass_heading << endl;
  m_msgs << "Compass heading: " << m_compass_heading << endl;

  return(true);
}

bool depthFirstSearch(string name, map<string, map<string, BlockStatus>> &block_matrix, vector<string> &visited, vector<string> &recursive_stack){
    // Mark the current node as visited and add to the recursion stack
    visited.push_back(name);
    recursive_stack.push_back(name);

    // Iterate through all the nodes that the current node is blocked by
    for (auto const& blockage : block_matrix[name]) {
      if (blockage.first == "total") {
        continue;
      }
      
      if (blockage.first == "name") { //Should not be necessary, as a node cannot be blocked by itself
        continue;
      }

      if (blockage.second == PHYSICALLY_BLOCKED){
        bool already_visited = find(visited.begin(), visited.end(), blockage.first) != visited.end();
        if (!already_visited && depthFirstSearch(blockage.first, block_matrix, visited, recursive_stack)){
          return true;
        } else if (find(recursive_stack.begin(), recursive_stack.end(), blockage.first) != recursive_stack.end()){
          return true;
        }
      }
    }

    //Remove the node "name" from the recursion stack
    recursive_stack.erase(remove(recursive_stack.begin(), recursive_stack.end(), name), recursive_stack.end());

    return false;
}

bool isPermablocked(std::string name, std::map<std::string, std::map<std::string, BlockStatus>> &block_matrix, std::map<std::string, XYSegList> &trajectory_map){
  bool permablocked = false;

  for (auto const& blockage : block_matrix[name]) {
    if (blockage.first == "total") {
      continue;
    }

    if (blockage.first == "name") { //Should not be necessary, as a node cannot be blocked by itself
      continue;
    }

    if (blockage.second == PHYSICALLY_BLOCKED){
      int points_left_blocker = int(trajectory_map[blockage.first].size());
      if (points_left_blocker <= 1) {
        permablocked = true;
        break;
      }
    }
  }

  return permablocked;
}