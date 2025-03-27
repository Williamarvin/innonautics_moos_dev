/************************************************************/
/*    NAME: Filip Stromstad                                 */
/*    ORGN: MIT                                             */
/*    FILE: BHV_DubinsPath.cpp                              */
/*    DATE: 03.13.2024                                      */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_DubinsPath.h"
#include "dubin.h"
#include "XYFormatUtilsSegl.h"
#include "XYFormatUtilsPoint.h"
#include "XYFormatUtilsPoly.h"
#include "OF_Coupler.h"
#include "ZAIC_PEAK.h"
#include "AngleUtils.h"
#include "NodeMessage.h"
#include "XYVector.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_DubinsPath::BHV_DubinsPath(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "dubins_path");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING");
  addInfoVars("COMPASS_HEADING_RAW", "no_warning");
  // addInfoVars("COMPASS_HEADING_RAW");
  addInfoVars("DEMUSTER_CONFIG");
  addInfoVars("DEADLOCK", "no_warning");

  // Set default values for the configuration parameters
  m_goal_heading = 0;
  m_goal_x = 0;
  m_goal_y = 0;
  m_r1 = 1;
  m_r2 = 1;
  m_r3 = 1;

  m_precision = 1;
  m_speed_desired = 1.0;
  m_speed_previous = 0;
  m_capture_radius = 2;
  m_slip_radius = 3;
  m_drift_radius = 0.1;
  m_drift_heading_thresh = 45;
  m_project_first_point_by_capture_radius = false;
  m_speed_LPF_alpha = 0.1;

  m_visualize_path_idle = false; 
  m_slowdown_range = -1; //Deactivated by default

  m_op_region = XYPolygon();

  //Set default values for the state variables
  m_osx = 0;
  m_osy = 0;
  m_osh = 0;
  m_osh_comp = 0;
  m_compass_declination = 0;
  m_use_compass_heading = false;

  m_trajectory = XYSegList();
  m_nextpt = XYPoint();
  m_nextnextpt = XYPoint();
  m_lastnextpt = XYPoint();
  m_curr_ix = 0;
  m_current_cpa = -1;

  m_optimal_path_length = -1;

  m_path_generated = false;
  m_number_of_paths_generated = 0;

  m_idle_time = -1; 

  m_only_right_turns = false;
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_DubinsPath::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "goal_heading") && isNumber(val)) {
    m_goal_heading = double_val;
    return(true);
  }
  else if ((param == "goal_x") && isNumber(val)) {
    m_goal_x = double_val;
    return(true);
  }
  else if ((param == "goal_y") && isNumber(val)) {
    m_goal_y = double_val;
    return(true);
  }
  else if (param == "goal_point"){
    XYPoint goal_point = string2Point(val);
    m_goal_x = goal_point.x();
    m_goal_y = goal_point.y();
    return(true);
  }
  else if (param == "vname"){
    m_os_name = val;
    return(true);
  }
  else if ((param == "r") && isNumber(val) && (double_val >= 0)){ //Handle if both r and one of r1, r2, r3 are set
    m_r1 = double_val;
    m_r2 = double_val;
    m_r3 = double_val;
    return(true);
  }
  else if ((param == "r1") && isNumber(val) && (double_val >= 0)){
    m_r1 = double_val;
    return(true);
  }
  else if ((param == "r2") && isNumber(val) && (double_val >= 0)){ 
    m_r2 = double_val;
    return(true);
  }
  else if ((param == "r3") && isNumber(val) && (double_val >= 0)) {
    m_r3 = double_val;
    return(true);
  }
  else if ((param == "precision") && isNumber(val) && (double_val >= 0)) {
    m_precision = double_val;
    return(true);
  }
  else if ((param == "speed") && isNumber(val) && (double_val >= 0)) {
    m_speed_desired = double_val;
    return(true);
  }
  else if ((param == "capture_radius") && isNumber(val) && (double_val >= 0)) {
    m_capture_radius = double_val;
    return(true);
  }
  else if ((param == "slip_radius") && isNumber(val) && (double_val >= 0)) {
    m_slip_radius = double_val;
    return(true);
  }
  else if ((param == "drift_radius") && isNumber(val) && (double_val >= 0)) {
    m_drift_radius = double_val;
    return(true);
  }
  else if ((param == "drift_heading") && isNumber(val) && (double_val >= 0)) {
    m_drift_heading_thresh = double_val;
    return(true);
  }
  else if ((param == "project_first_point") && (val == "true" || val == "false")) {
    m_project_first_point_by_capture_radius = (val == "true");
    return(true);
  }
  else if ((param == "visualize_path_idle") && (val == "true" || val == "false")) {
    m_visualize_path_idle = (val == "true");
    return(true);
  }
  else if ((param == "slowdown_range") && isNumber(val) && (double_val >= 0)) {
    m_slowdown_range = double_val;
    return(true);
  } 
  else if (param == "regenerate_path"){
    m_path_generated = false;
    return(true);
  } 
  else if (param == "op_region"){
    XYPolygon new_op_region = string2Poly(val);
    if(!new_op_region.is_convex())  // Should be convex - false otherwise
      return(false);
    m_op_region = new_op_region;
    return(true);
  }
  else if (param == "only_right_turns"){
    m_only_right_turns = (val == "true");
    return(true);
  }
  else if (param == "compass_declination"){
    m_compass_declination = double_val;
    return(true);
  }
  else if (param == "use_compass_heading"){
    m_use_compass_heading = (val == "true");
    return(true);
  }
  else if (param == "speed_lpf_alpha"){
    m_speed_LPF_alpha = double_val;
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

void BHV_DubinsPath::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_DubinsPath::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_DubinsPath::onIdleState()
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
  output_seglist.set_label(m_os_name + "_dubin_" + to_string(m_number_of_paths_generated+1));
  
  NodeMessage node_msg;
  node_msg.setSourceNode(m_os_name);
  node_msg.setDestNode("all");
  node_msg.setVarName("TRAJECTORY_MSG");
  node_msg.setStringVal(output_seglist.get_spec());
  postMessage("NODE_MESSAGE_LOCAL", node_msg.getSpec());

  if(!m_visualize_path_idle) {
    return;
  }
  
  //Translate from compass rose in degrees to radians
  double goal_heading_rad = (90 - m_goal_heading) * M_PI / 180; 
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

  return;
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_DubinsPath::onCompleteState()
{
  // Clean-up any visualizations
  postMessage("VIEW_SEGLIST", m_trajectory.get_spec_inactive());
  m_trajectory = XYSegList();
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_DubinsPath::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_DubinsPath::onIdleToRunState()
{
  m_path_generated = false;

  return;
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_DubinsPath::onRunToIdleState()
{
  //Clear any visualizations
  postMessage("VIEW_SEGLIST", m_trajectory.get_spec_inactive());
  m_trajectory = XYSegList();

  //Send current position...want to replace this with node reports!!!
  XYSegList output_seglist;
  output_seglist.add_vertex(m_osx, m_osy);
  output_seglist.set_label(m_us_name + "_dubin_" + to_string(m_number_of_paths_generated));
  postMessage("TRAJECTORY_MSG_LOCAL", output_seglist.get_spec());
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_DubinsPath::onRunState()
{
  // ---Update the configuration parameters
  updateConfigParameters();

  bool ok;
  string deadlocked = getBufferStringVal("DEADLOCK", ok);
  if(ok && deadlocked == "1" && !m_deadlock){
    m_deadlock = true;
    // m_only_right_turns = true;
    // m_path_generated = false;
  } else if (ok && deadlocked == "0" && m_deadlock){
    m_deadlock = false;
    // m_only_right_turns = false;
    // m_path_generated = false;
  }

  // if (deadlocked == "1"){
  //   postMessage("MOOS_MANUAL_OVERRIDE", "true");
  //   postRepeatableMessage("DESIRED_THRUST", -40);
  //   return 0;
  // }

  // ---Step 1: Get ownship position and heading
  bool ok1, ok2, ok3;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  m_osh = getBufferDoubleVal("NAV_HEADING", ok3);
  
  if (!ok1 || !ok2 || !ok3){
    postWMessage("No ownship X/Y/Heading info in buffer.");
    return 0;
  }

  if (m_use_compass_heading){
    bool ok4;
    m_osh_comp = getBufferDoubleVal("COMPASS_HEADING_RAW", ok4);
    // if (!ok4){
    //   postWMessage("No compass heading raw info in buffer.");
    //   return 0;
    // } 
    m_osh_comp += m_compass_declination;
    if (m_osh_comp < 0){
      m_osh_comp += 360;
    } else if (m_osh_comp >= 360){
      m_osh_comp -= 360;
    }
  }

  double vector_angle = 0;
  if (m_use_compass_heading){
    vector_angle = m_osh_comp;
  } else {
    vector_angle = m_osh;
  }

  // XYVector os_heading_vec = XYVector(m_osx, m_osy, 5, vector_angle);
  // os_heading_vec.set_label(m_us_name + "_heading");
  // os_heading_vec.setHeadSize(1);
  // os_heading_vec.set_color("edge", "magenta");
  // os_heading_vec.set_color("fill", "magenta");
  // os_heading_vec.set_color("vertex", "magenta");
  // postMessage("VIEW_VECTOR", os_heading_vec.get_spec());


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

    m_trajectory = new_trajectory;
    m_path_generated = true;
    m_number_of_paths_generated++;

    m_trajectory.set_label(m_us_name + "_dubin");
    postMessage("VIEW_SEGLIST", m_trajectory.get_spec());

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

  // double an = 90 - heading_desired * 180 / M_PI;
  // XYVector heading_vec_test = XYVector(m_nextpt.x(), m_nextpt.y(), 5, an);
  // heading_vec_test.set_label(m_us_name + "_des");
  // heading_vec_test.setHeadSize(1);
  // heading_vec_test.set_color("edge", "red");
  // heading_vec_test.set_color("fill", "red");
  // heading_vec_test.set_color("vertex", "red");
  // postMessage("VIEW_VECTOR", heading_vec_test.get_spec());
  

  // (m_current_cpa == -1) indicates first time this function called
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
  output_seglist.add_vertex(m_osx, m_osy); //Add current position
  output_seglist.set_label(m_us_name + "_dubin_" + to_string(m_number_of_paths_generated));
  for (int i = m_curr_ix; i < m_trajectory.size(); i++){ //Add remaining waypoints
    output_seglist.add_vertex(m_trajectory.get_vx(i), m_trajectory.get_vy(i));
  }
  postMessage("TRAJECTORY_MSG_LOCAL", output_seglist.get_spec());

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
IvPFunction *BHV_DubinsPath::buildOF() {
  IvPFunction *ipf = 0;

  // ---Build IvP function for speed
  IvPFunction *spd_ipf = 0;
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  double cruise_speed = m_speed_desired;

  if (m_speed_desired > m_speed_previous){ //Accelerating --> LPF to smooth
    double next_speed = m_speed_LPF_alpha * m_speed_desired + (1 - m_speed_LPF_alpha) * m_speed_previous;
    cruise_speed = next_speed;
    m_speed_previous = next_speed; 
  } else {
    m_speed_previous = m_speed_desired;
  }
  
  //Add slowdown when approaching last waypoint
  if (m_slowdown_range > 0 && m_speed_desired > 0.1){
    int slowdowon_range_points = round(m_slowdown_range / m_precision);
    if (m_trajectory.size() - m_curr_ix < slowdowon_range_points){
      cruise_speed = max(0.1, m_speed_desired * (0 + 1.0*(m_trajectory.size() - m_curr_ix) / slowdowon_range_points));
    }
  }

  double peak_width = cruise_speed / 2;
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
  
  int ix = crs_zaic.addComponent();
  crs_zaic.setParams(m_osh, 30, 180, 5, 0, 20, ix);
  // TODO: also potentially use compass heading here? Understand this...
  
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


void BHV_DubinsPath::updateConfigParameters(){
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