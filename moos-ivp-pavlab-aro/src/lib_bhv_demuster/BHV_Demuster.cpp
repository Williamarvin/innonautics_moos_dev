/************************************************************/
/*    NAME: Filip Stromstad                                 */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Demuster.cpp                              */
/*    DATE: 03.13.2024                                      */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "BHV_Demuster.h"
#include "dubin.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "BuildUtils.h"
#include "NodeMessage.h"
#include "XYFormatUtilsSegl.h"
#include "XYFormatUtilsPoint.h"
#include "XYFormatUtilsPoly.h"
#include "OF_Coupler.h"
#include "ZAIC_PEAK.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_Demuster::BHV_Demuster(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "dubins_path");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING", "NAV_SPEED");
  addInfoVars("COMPASS_HEADING_RAW", "no_warning");
  addInfoVars("DEMUSTER_CONFIG");
  addInfoVars("TRAJECTORY_MSG, TRAJECTORY_HANDSHAKE, SPEED_MSG");

  // Set default values for the configuration parameters
  // Configuration parameters - Dubin trajectory
  m_goal_heading = 0;
  m_goal_x = 0;
  m_goal_y = 0;
  m_r1 = 1;
  m_r2 = 1;
  m_r3 = 1;
  m_precision = 1;
  m_op_region = XYPolygon();
  m_visualize_path_idle = false; 
  m_project_first_point_by_capture_radius = false;
  m_only_right_turns = false;

  // Configuration parameters - Waypoint following
  m_capture_radius = 2;
  m_slip_radius = 3;
  m_drift_radius = 0.1;
  m_drift_heading_thresh = 45;

  // Configuration parameters - Speed policy
  m_speed_default = 1.0;
  m_speed_desired = 0;
  m_speed_previous = 0;
  m_speed_LPF_alpha = 0.1;

  m_max_safety_dist = 4;
  m_min_safety_dist = 2;
  m_max_horizon_meters = 10;
  m_min_horizon_meters = 2;

  m_slowdown_range = -1; //Deactivated by default
  m_use_dynamic_speed = false;
  m_synchronize = false;
  m_turn_in_place = false;
  m_reverse_thrust = -30;

  m_mode = "decluster"; //decluster, simultaneous, sequential
  m_seq_counter = 0;

  m_show_visualization = false;

  //Set default values for the state variables
  // State variables - Ownship (os)
  m_osx = 0;
  m_osy = 0;
  m_os_speed = 0;
  m_osh = 0;
  m_osh_comp = 0;
  m_use_compass_heading = false;

  // State variables - Current trajectory
  m_trajectory = XYSegList();
  m_curr_ix = 0;
  m_current_cpa = -1;
  m_nextpt = XYPoint();
  m_nextnextpt = XYPoint();
  m_lastnextpt = XYPoint();
  m_path_generated = false;
  m_number_of_paths_generated = 0;

  // State variables - Block matrix
  m_trajectory_map = {};
  m_speed_map = {};
  m_block_matrix = {};
  m_temp_block_dist_matrix = {};
  m_temp_block_limit = 5;

  m_handshakes = {};
  m_nodes_in_proximity = {};
  m_proximity_range = 10;
  m_os_trajectory_number = 0;
  m_known_traj_numbers = {};

  // State variables - Timers  
  m_potential_permablock_timer = -1;
  m_potential_deadlock_timer = -1;
  m_deadlock_timer = -1;

}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Demuster::setParam(string param, string val)
{ 
  param = tolower(param); // Convert the parameter to lower case for more general matching
  double double_val = atof(val.c_str()); // Get the numerical value of the param argument for convenience once
  
  if(param == "regenerate_path") {
    m_path_generated = false;
    return(true);
  // Configuration parameters - Dubin trajectory
  } else if ((param == "goal_heading") && isNumber(val)) {
    m_goal_heading = double_val;
    return(true);
  } else if ((param == "goal_x") && isNumber(val)) {
    m_goal_x = double_val;
    return(true);
  } else if ((param == "goal_y") && isNumber(val)) {
    m_goal_y = double_val;
    return(true);
  } else if ((param == "goal_point")){
    XYPoint goal_point = string2Point(val);
    m_goal_x = goal_point.x();
    m_goal_y = goal_point.y();
    return(true);
  } else if ((param == "r") && isNumber(val) && (double_val >= 0)){
    m_r1 = double_val;
    m_r2 = double_val;
    m_r3 = double_val;
    return(true);
  } else if ((param == "r1") && isNumber(val) && (double_val >= 0)){
    m_r1 = double_val;
    return(true);
  } else if ((param == "r2") && isNumber(val) && (double_val >= 0)){ 
    m_r2 = double_val;
    return(true);
  } else if ((param == "r3") && isNumber(val) && (double_val >= 0)) {
    m_r3 = double_val;
    return(true);
  } else if ((param == "precision") && isNumber(val) && (double_val >= 0)) {
    m_precision = double_val;
    return(true);
  } else if ((param == "op_region")){
    XYPolygon new_op_region = string2Poly(val);
    if(!new_op_region.is_convex())  // Should be convex - false otherwise
      return(false);
    m_op_region = new_op_region;
    return(true);
  } else if ((param == "visualize_path_idle") && (val == "true" || val == "false")) {
    m_visualize_path_idle = (val == "true");
    return(true);
  } else if ((param == "project_first_point") && (val == "true" || val == "false")) {
    m_project_first_point_by_capture_radius = (val == "true");
    return(true);
  } else if ((param == "only_right_turns") && (val == "true" || val == "false")) {
    m_only_right_turns = (val == "true");
    return(true);
  // Configuration parameters - Waypoint following
  } else if ((param == "capture_radius") && isNumber(val) && (double_val >= 0)) {
    m_capture_radius = double_val;
    return(true);
  } else if ((param == "slip_radius") && isNumber(val) && (double_val >= 0)) {
    m_slip_radius = double_val;
    return(true);
  } else if ((param == "drift_radius") && isNumber(val) && (double_val >= 0)) {
    m_drift_radius = double_val;
    return(true);
  } else if ((param == "drift_heading") && isNumber(val) && (double_val >= 0)) {
    m_drift_heading_thresh = double_val;
    return(true);
  // Configuration parameters - Speed policy
  } else if ((param == "default_speed") && isNumber(val) && (double_val >= 0)) {
    m_speed_default = double_val;
    return(true);
  } else if ((param == "speed") && isNumber(val) && (double_val >= 0)) {
    m_speed_desired = double_val;
    return(true);
  } else if ((param == "speed_lpf_alpha") && isNumber(val) && (double_val >= 0) && (double_val <= 1)) {
    m_speed_LPF_alpha = double_val;
    return(true);
  } else if ((param == "max_safety_distance") && isNumber(val) && (double_val >= 0)) {
    m_max_safety_dist = double_val;
    return(true);
  } else if ((param == "min_safety_distance") && isNumber(val) && (double_val >= 0)) {
    m_min_safety_dist = double_val;
    return(true);
  } else if ((param == "max_horizon") && isNumber(val) && (double_val >= 0)) {
    m_max_horizon_meters = double_val;
    return(true);
  } else if ((param == "min_horizon") && isNumber(val) && (double_val >= 0)) {
    m_min_horizon_meters = double_val;
    return(true);
  } else if ((param == "slowdown_range") && isNumber(val) && (double_val >= 0)) {
    m_slowdown_range = double_val;
    return(true);
  } else if ((param == "use_dynamic_speed") && (val == "true" || val == "false")) {
    m_use_dynamic_speed = (val == "true");
    return(true);
  } else if ((param == "synchronize") && (val == "true" || val == "false")) {
    m_synchronize = (val == "true");
    return(true);
  } else if ((param == "turn_in_place") && (val == "true" || val == "false")) {
    m_turn_in_place = (val == "true");
    return(true);
  } else if ((param == "reverse_thrust") && isNumber(val)) {
    m_reverse_thrust = double_val;
    return(true);
  } else if ((param == "show_visualization") && (val == "true" || val == "false")) {
    m_show_visualization = (val == "true");
    return(true);
  // Configuration parameters - States
  } else if ((param == "use_compass_heading") && (val == "true" || val == "false")) {
    m_use_compass_heading = (val == "true");
    return(true);
  } else if ((param == "temp_block_limit") && isNumber(val) && (double_val >= 0)) {
    m_temp_block_limit = double_val;
    return(true);
  } else if ((param == "proximity_range") && isNumber(val) && (double_val >= 0)) {
    m_proximity_range = double_val;
    return(true);
  } else if (param == "mode" && (val == "decluster" || val == "simultaneous" || val == "sequential" || val == "deploy")) {
    m_mode = val;
    return(true);
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_Demuster::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_Demuster::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_Demuster::onIdleState()
{
  // ---Update the configuration parameters
  updateConfigParameters();

    bool ok1, ok2, ok3;

    m_osx = getBufferDoubleVal("NAV_X", ok1);
    m_osy = getBufferDoubleVal("NAV_Y", ok2);
    m_osh = getBufferDoubleVal("NAV_HEADING", ok3);
    if (!ok1 || !ok2 || !ok3){
      postWMessage("No ownship X/Y/Heading info in buffer.");
      return;
    }

  //Make sure to always send your own position
  XYSegList output_seglist;
  output_seglist.add_vertex(m_osx, m_osy);
  output_seglist.set_label(m_us_name + "_dubin_" + to_string(m_number_of_paths_generated+1));
  
  NodeMessage node_msg;
  node_msg.setSourceNode(m_us_name);
  node_msg.setDestNode("all");
  node_msg.setVarName("TRAJECTORY_MSG");
  node_msg.setStringVal(output_seglist.get_spec());
  postMessage("NODE_MESSAGE_LOCAL", node_msg.getSpec());

  if(m_visualize_path_idle) {
    double goal_heading_rad = (90 - m_goal_heading) * M_PI / 180; //Translate from compass rose in degrees to radians
    double os_heading_rad = 0;
      if (m_use_compass_heading){
        os_heading_rad = (90 - m_osh_comp) * M_PI / 180;
      } else {
        os_heading_rad = (90 - m_osh) * M_PI / 180;
      }

    DubinsPath dp = DubinsPath();

    Point start_point = Point(m_osx, m_osy);
    if (m_project_first_point_by_capture_radius){
      start_point = Point(m_osx + m_capture_radius * cos(os_heading_rad), m_osy + m_capture_radius * sin(os_heading_rad));
    }

    string trajectory_str = dp.findOptimalWaypoints(start_point, os_heading_rad, Point(m_goal_x, m_goal_y), goal_heading_rad, m_r1, m_r2, m_r3, m_precision);
    XYSegList new_trajectory = string2SegList(trajectory_str);
    new_trajectory.set_label(m_us_name + "_dubin");

    postMessage("VIEW_SEGLIST", new_trajectory.get_spec());
  }
  

  return;
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Demuster::onCompleteState()
{
  // Clean-up any visualizations
  if (m_show_visualization){
    postMessage("VIEW_SEGLIST", m_trajectory.get_spec_inactive());
  }
  m_trajectory = XYSegList();
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_Demuster::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_Demuster::onIdleToRunState()
{
  m_path_generated = false;

  return;
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_Demuster::onRunToIdleState()
{
  //Clear any visualizations
  if (m_show_visualization){
    postMessage("VIEW_SEGLIST", m_trajectory.get_spec_inactive());
  }
  m_trajectory = XYSegList();

  //Send current position...want to replace this with node reports!!!
  XYSegList output_seglist;
  output_seglist.add_vertex(m_osx, m_osy);
  output_seglist.set_label(m_us_name + "_dubin_" + to_string(m_number_of_paths_generated));
  postMessage("TRAJECTORY_MSG_LOCAL", output_seglist.get_spec()); //TODO: Will be removed

  NodeMessage node_msg;
  node_msg.setSourceNode(m_us_name);
  node_msg.setDestNode("all");
  node_msg.setVarName("TRAJECTORY_MSG");
  node_msg.setStringVal(output_seglist.get_spec());
  postMessage("NODE_MESSAGE_LOCAL", node_msg.getSpec());
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_Demuster::onRunState()
{
  // ---Update the configuration parameters
  updateConfigParameters();

  // ---Step 1: Fetch new mail
  bool ok = fetchNewMail();
  if (!ok){
    postWMessage("Error fetching new mail.");
    return 0;
  }

  // ---Step 1.5: Generate path if not already generated
  if (!m_path_generated){
    //Translate from compass rose in degrees to radians
    double goal_heading_rad = (90 - m_goal_heading) * M_PI / 180; 
    // double os_heading_rad = (90 - m_osh) * M_PI / 180;
    double os_heading_rad = 0;
    if (m_use_compass_heading){
      os_heading_rad = (90 - m_osh_comp) * M_PI / 180;
    } else {
      os_heading_rad = (90 - m_osh) * M_PI / 180;
    }

    DubinsPath dp = DubinsPath();
    XYSegList new_trajectory;


    Point start_point = Point(m_osx, m_osy);
    if (m_project_first_point_by_capture_radius){
      start_point = Point(m_osx + m_capture_radius * cos(os_heading_rad), m_osy + m_capture_radius * sin(os_heading_rad));
    }

    if (m_op_region.size() == 0){ //No op region
      vector<string> illegal_paths = {};
      if (m_only_right_turns){
        //Only allow the first turn to be a right turn
        illegal_paths = {"LSL", "LSR", "LRL"};
        // Only restrict right turns on the first path
        m_only_right_turns = false;
      }
      string trajectory_str = dp.findOptimalWaypoints(start_point, os_heading_rad, Point(m_goal_x, m_goal_y), goal_heading_rad, m_r1, m_r2, m_r3, m_precision, illegal_paths);
      new_trajectory = string2SegList(trajectory_str);
      
    } else { //There is an op region
      vector<string> illegal_paths = {};
      bool path_outside_op_region = true;
      while (path_outside_op_region){
        path_outside_op_region = false; //Assume path is inside op region
        string trajectory_str = dp.findOptimalWaypoints(start_point, os_heading_rad, Point(m_goal_x, m_goal_y), goal_heading_rad, m_r1, m_r2, m_r3, m_precision, illegal_paths);
        new_trajectory = string2SegList(trajectory_str);

        for (int i = 0; i < new_trajectory.size(); i++){
          if (!m_op_region.contains(new_trajectory.get_vx(i), new_trajectory.get_vy(i))){
            path_outside_op_region = true;
            illegal_paths.push_back(dp.m_type);
            break;
          }
        }
      }
    }

    if (new_trajectory.size() == 0){
      postWMessage("No path found.");
      return 0;
    }

    new_trajectory.set_label(m_us_name + "_dubin");
    m_trajectory = new_trajectory;
    m_trajectory.set_label_color("invisible");
    m_trajectory.set_vertex_color("invisible");
    m_trajectory_map[m_us_name] = m_trajectory;
    m_path_generated = true;
    m_number_of_paths_generated++;
    m_handshakes.clear();

    if (m_show_visualization){
      postMessage("VIEW_SEGLIST", m_trajectory.get_spec());
    }

    m_curr_ix = 0;
    m_nextpt = m_trajectory.get_point(m_curr_ix);
    m_nextnextpt = m_trajectory.get_point(m_curr_ix + 1);
    m_lastnextpt = string2Point(dp.m_waypoint_extra);
    // postMessage("VIEW_POINT", m_lastnextpt.get_spec()); 
  }

  // ---Step 2: Check if we have reached the next point
  double dist_to_nextpt = sqrt(pow(m_nextpt.x() - m_osx, 2) + pow(m_nextpt.y() - m_osy, 2));
  
  // TODO: Rydd opp i denne koden...
  Point next_to_nextnext_vector = Point(m_nextnextpt.x() - m_nextpt.x(), m_nextnextpt.y() - m_nextpt.y());
  XYPoint prevpt = m_trajectory.get_point(m_curr_ix - 1);
  Point prev_to_next_vector = Point(m_nextpt.x() - prevpt.x(), m_nextpt.y() - prevpt.y());
  // double heading_desired = atan2(next_to_nextnext_vector.y, next_to_nextnext_vector.x);
  double heading_desired = 0;
  if (m_curr_ix > 2){
    heading_desired = atan2(prev_to_next_vector.y, prev_to_next_vector.x);
  } else {
    heading_desired = atan2(next_to_nextnext_vector.y, next_to_nextnext_vector.x);
  }

  double heading_rad = 0;
  if (m_use_compass_heading){
    heading_rad = (90 - m_osh_comp) * M_PI / 180;
  } else {
    heading_rad = (90 - m_osh) * M_PI / 180;
  }

  if (heading_desired < 0){
    heading_desired += 2*M_PI;
  }
  if (heading_rad < 0){
    heading_rad += 2*M_PI;
  }

  double heading_diff = fabs(heading_desired - heading_rad) / M_PI * 180;
  if (heading_diff > 180){
    heading_diff = 360 - heading_diff;
  }
  
  if((m_current_cpa == -1) || (dist_to_nextpt < m_current_cpa))
    m_current_cpa = dist_to_nextpt;

  if (dist_to_nextpt < m_capture_radius){ //Captured the next point
    m_curr_ix++;
    m_current_cpa = -1;
    if (m_curr_ix >= m_trajectory.size()){
      setComplete();
      return 0;
    }
    m_nextpt = m_trajectory.get_point(m_curr_ix);
    if (m_curr_ix + 1 >= m_trajectory.size()){
      m_nextnextpt = m_lastnextpt;
    } else {
      m_nextnextpt = m_trajectory.get_point(m_curr_ix + 1);
    }
  } else if ((dist_to_nextpt > m_current_cpa) && (m_current_cpa <= m_slip_radius)){ //Increased distance to point and we are within the slip radius
    //...same as when we have captured the point
    m_curr_ix++;
    m_current_cpa = -1;
    if (m_curr_ix >= m_trajectory.size()){
      setComplete();
      return 0;
    }
    m_nextpt = m_trajectory.get_point(m_curr_ix);
    if (m_curr_ix + 1 >= m_trajectory.size()){
      m_nextnextpt = m_lastnextpt;
    } else {
      m_nextnextpt = m_trajectory.get_point(m_curr_ix + 1);
    }
  } else if ((dist_to_nextpt - m_current_cpa)  > m_drift_radius){ //No capture, no slip. Drifted more x meters from the next point
    //We have "fallen off" the trajectory...
    //...so we need to recalculate the path
    m_path_generated = false;
  } else if (heading_diff > m_drift_heading_thresh && ((m_curr_ix + 2) < m_trajectory.size())){ //Heading is off by more than x degrees
    //We have "fallen off" the trajectory...
    //...so we need to recalculate the path
    m_path_generated = false;
  }

  //-------Send name, current position and remaining waypoints to a MOOS variable:--------
  XYSegList output_seglist;
  output_seglist.set_label(m_us_name + "_dubin_" + to_string(m_number_of_paths_generated));
  output_seglist.add_vertex(m_osx, m_osy); //Add current position
  for (int i = m_curr_ix; i < m_trajectory.size(); i++){ //Add remaining waypoints
    output_seglist.add_vertex(m_trajectory.get_vx(i), m_trajectory.get_vy(i));
  }
  postMessage("TRAJECTORY_MSG_LOCAL", output_seglist.get_spec()); // TODO: Will be removed
  NodeMessage node_msg;
  node_msg.setSourceNode(m_us_name);
  node_msg.setDestNode("all");
  node_msg.setVarName("TRAJECTORY_MSG");
  node_msg.setStringVal(output_seglist.get_spec());
  postMessage("NODE_MESSAGE_LOCAL", node_msg.getSpec());

  int dubin_points_left = m_trajectory.size() - m_curr_ix;
  postMessage("DUBIN_POINTS_LEFT", dubin_points_left);
  

  // ---Step 3: Build the IvP function
  IvPFunction *ipf = buildOF();


  // ---Step 4: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}

//---------------------------------------------------------------
// Procedure: buildOF()
//   Purpose: Create Objective function to transition to next waypoint
IvPFunction *BHV_Demuster::buildOF() {
  IvPFunction *ipf = 0;

  // ---Build IvP function for speed
  IvPFunction *spd_ipf = 0;
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  double cruise_speed = m_speed_desired;

  if (m_mode == "simultaneous"){
    cruise_speed = m_speed_default;
  } 
  // else if (m_mode == "sequential"){
  //   int max_number_of_moving_vehicles = 1;
  //   int num_non_zero_speeds = 0;
  //   for (const auto &pair : m_speed_map){
  //     if (pair.first == m_us_name){
  //       continue;
  //     }

  //     if (pair.second > 0){
  //       num_non_zero_speeds++;
  //     }
  //   }

  //   if (num_non_zero_speeds >= max_number_of_moving_vehicles){
  //     cruise_speed = 0;
  //     m_seq_counter = 0;
  //   } else {
  //     // Might be allowed to run
  //     m_seq_counter++;

  //     char first_char = m_us_name[0];
  //     int first_char_int = first_char - 'a' + 1;
  //     if (m_seq_counter < first_char_int*25){
  //       cruise_speed = 0;
  //     } 
  //   }
  // }

  if (cruise_speed > m_speed_previous){ //Accelerating --> LPF to smooth
    double next_speed = m_speed_LPF_alpha * cruise_speed + (1 - m_speed_LPF_alpha) * m_speed_previous;
    cruise_speed = next_speed;
    m_speed_previous = next_speed; 
  } else {
    m_speed_previous = cruise_speed;
  }
  
  //Add slowdown when approaching last waypoint
  if (m_slowdown_range > 0 && cruise_speed > 0.1){
    int slowdowon_range_points = round(m_slowdown_range / m_precision);
    if (m_trajectory.size() - m_curr_ix < slowdowon_range_points){
      cruise_speed = max(0.1, cruise_speed * (0 + 1.0*(m_trajectory.size() - m_curr_ix) / slowdowon_range_points));
    }
  }

  double peak_width = cruise_speed / 2;
  // cruise_speed = 1;
  spd_zaic.setParams(cruise_speed, peak_width, 1.6, 20, 0, 100);
  spd_ipf = spd_zaic.extractIvPFunction();
  if(!spd_ipf) {
	  postWMessage("Failure on the SPD ZAIC via ZAIC_PEAK utility");
  }

  // ---Build IvP function for course
  // double rel_ang_to_nxt_pt = relAng(m_osx, m_osy, m_nextpt.x(), m_nextpt.y());
  double rel_ang_to_nxt_pt = relAng(m_osx, m_osy, m_nextnextpt.x(), m_nextnextpt.y());

  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setValueWrap(true);
  crs_zaic.setParams(rel_ang_to_nxt_pt, 0, 180, 50, 0, 100);
  
  // int ix = crs_zaic.addComponent();
  // crs_zaic.setParams(m_osh, 30, 180, 5, 0, 20, ix);
  
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);

  if(!crs_ipf) 
    postWMessage("Failure on the CRS ZAIC");

  // ---Coupling speed and course IvP functions
  OF_Coupler coupler;
  ipf = coupler.couple(crs_ipf, spd_ipf, 50, 50);
  if(!ipf){
    postWMessage("Failure on the CRS_SPD COUPLER");  
  }

  return ipf;   
}

bool BHV_Demuster::fetchNewMail(){
  // Handle "DEMUSTER_CONFIG" in updateConfigParameters()
  bool ok1, ok2, ok3, ok4;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  m_osh = getBufferDoubleVal("NAV_HEADING", ok3);
  m_os_speed = getBufferDoubleVal("NAV_SPEED", ok4);
  m_speed_map[m_us_name] = m_os_speed;
  
  if (!ok1 || !ok2 || !ok3 || !ok4){
    postWMessage("No ownship X/Y/Heading/Speed info in buffer.");
    return 0;
  }

  if (m_use_compass_heading){
    bool ok5;
    m_osh_comp = getBufferDoubleVal("COMPASS_HEADING_RAW", ok5);

    if (m_osh_comp < 0){
      m_osh_comp += 360;
    } else if (m_osh_comp >= 360){
      m_osh_comp -= 360;
    }

    if (!ok5){
      postWMessage("No compass heading info in buffer.");
      return 0;
    }
  }

  // bool ok6;
  // string speedMSG = getBufferStringVal("SPEED_MSG", ok6);
  // if (!ok6){
  //   postWMessage("No speed message in buffer.");
  //   return 0;
  // }
  // vector<string> parts = parseString(speedMSG, ',');
  // m_speed_map[parts[0]] = stod(parts[1]);


  return 1;
}

void BHV_Demuster::updateConfigParameters(){
  if (getBufferVarUpdated("DEMUSTER_CONFIG")){
    string new_config = getBufferStringVal("DEMUSTER_CONFIG");
    if (new_config == "turning_radius_increase"){
      m_r1 += 1;
      m_r2 += 1;
      m_r3 += 1;
      postMessage("AA_DEMUSTER_CONFIG_UPDATED", "Turning radius: " + to_string(m_r1));
      m_path_generated = false;
    } 
    else if (new_config == "turning_radius_decrease"){
      m_r1 -= 1;
      m_r2 -= 1;
      m_r3 -= 1;
      postMessage("AA_DEMUSTER_CONFIG_UPDATED", "Turning radius: " + to_string(m_r1));
      m_path_generated = false;
    }
    else if (new_config == "slowdown_range_increase"){
      m_slowdown_range += 1;
      postMessage("AA_DEMUSTER_CONFIG_UPDATED", "Slowdown range: " + to_string(m_slowdown_range));
    }
    else if (new_config == "slowdown_range_decrease"){
      m_slowdown_range -= 1;
      postMessage("AA_DEMUSTER_CONFIG_UPDATED", "Slowdown range: " + to_string(m_slowdown_range));
    }
  }
}


// bool depthFirstSearch(string name, map<string, map<string, BlockStatus>> &block_matrix, vector<string> &visited, vector<string> &recursive_stack){
//     // Mark the current node as visited and add to the recursion stack
//     visited.push_back(name);
//     recursive_stack.push_back(name);

//     // Iterate through all the nodes that the current node is blocked by
//     for (auto const& blockage : block_matrix[name]) {
//       if (blockage.first == "total") {
//         continue;
//       }
      
//       if (blockage.first == "name") { //Should not be necessary, as a node cannot be blocked by itself
//         continue;
//       }

//       if (blockage.second == PHYSICALLY_BLOCKED){
//         bool already_visited = find(visited.begin(), visited.end(), blockage.first) != visited.end();
//         if (!already_visited && depthFirstSearch(blockage.first, block_matrix, visited, recursive_stack)){
//           return true;
//         } else if (find(recursive_stack.begin(), recursive_stack.end(), blockage.first) != recursive_stack.end()){
//           return true;
//         }
//       }
//     }

//     //Remove the node "name" from the recursion stack
//     recursive_stack.erase(remove(recursive_stack.begin(), recursive_stack.end(), name), recursive_stack.end());

//     return false;
// }


// bool isPermablocked(std::string name, std::map<std::string, std::map<std::string, BlockStatus>> &block_matrix, std::map<std::string, XYSegList> &trajectory_map){
//   bool permablocked = false;

//   for (auto const& blockage : block_matrix[name]) {
//     if (blockage.first == "total") {
//       continue;
//     }

//     if (blockage.first == "name") { //Should not be necessary, as a node cannot be blocked by itself
//       continue;
//     }

//     if (blockage.second == PHYSICALLY_BLOCKED){
//       int points_left_blocker = int(trajectory_map[blockage.first].size());
//       if (points_left_blocker <= 1) {
//         permablocked = true;
//         break;
//       }
//     }
//   }

//   return permablocked;
// }