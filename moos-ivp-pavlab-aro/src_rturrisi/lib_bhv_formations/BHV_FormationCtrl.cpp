/************************************************************/
/*    NAME: .                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_FormationCtrl.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "BHV_FormationCtrl.h"
#include "NodeRecordUtils.h"
#include "AngleUtils.h"
#include "NodeMessage.h" // In the lib_ufield library
#include "CtrlPointQueue.h"
#include "XYSegList.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_FormationCtrl::BHV_FormationCtrl(IvPDomain domain) : IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "FormationCtrl");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for

  m_us_name = "larry";
  m_debug = false;
}

double BHV_FormationCtrl::sat(double r_sat, double lambda, double r)
{
  if (r < r_sat)
  {
    return 1;
  }
  else
  {
    return exp(-lambda * (r - r_sat));
  }
}

bool BHV_FormationCtrl::dbg_print(const char *format, ...)
{
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    m_cfile = fopen(m_debug_fname.c_str(), "a");
    if (m_cfile != nullptr)
    {
      vfprintf(m_cfile, format, args);
      fclose(m_cfile);
      return true;
    }
    else
    {
      return false;
    }
  }
  return false;
}

void BHV_FormationCtrl::initStates()
{

  // Logic
  m_pos_idx = -1;
  m_formation_type = "vee";
  m_num_agents = 1;

  // Control
  m_desired_speed = 1;
  m_osx = 0;
  m_osy = 0;
  m_osh = 0;
  m_latest_r_reference = 0;
  m_os_node_record = NodeRecord();
  m_agent_states.clear();
  m_max_ctrl_queue_length = 5;
  m_min_ctrl_point_sep = 0.5;
  m_capture_radius = 1;
  m_slip_radius = 1;

  //// Local control gains
  m_kp_spd_1 = 0.5;
  m_kp_spd_2 = 0.2;
  m_kd_spd_1 = 0.5;
  m_kd_spd_2 = 0.2;
  m_kp_hdg_1 = 0;
  m_kp_hdg_2 = 0;
  m_kd_hdg_1 = 0;
  m_kd_hdg_2 = 0;

  //// Coupling gains
  m_kpc_spd = 0.25;
  m_kdc_spd = 0.1;
  m_kpc_hdg = 0;
  m_kdc_hdg = 0;

  //// Region of attraction and saturation gains
  m_rs_spd = 1;              // meters
  m_rs_hdg = 1;              // meters
  m_lambda_roa_spd_sr = 0.3; // exponential decay rate
  m_lambda_roa_hdg_sr = 0.3; // exponential decay rate

  // Representations
  m_os_state = AgentStateInfo();
  m_os_state_prev = AgentStateInfo();

  // Timing
  m_nr_t = 0.1;
  m_nr_t_prev = 0;

  // Utility
  m_idx_to_agent_name.clear();
  m_agent_name_to_idx.clear();
  m_max_speed = 1.8;
}

bool BHV_FormationCtrl::setParallelFormationType(std::string formation_type, std::string params)
{
  std::string formation_args = params;

  std::vector<std::string> args = parseString(formation_args, ',');

  for (auto arg : args)
  {
    std::string key = biteString(arg, '=');
    std::string value = arg;
    if (m_formation_geomgr.m_formation_specific_allowable_arguments[m_formation_type].count(key) > 0)
    {
      m_formation_params[key] = value;
      dbg_print("Key: %s | Value: %s\n", key.c_str(), value.c_str());
    }
    else
    {
      dbg_print("Invalid argument for formation type %s: %s\n", m_formation_type.c_str(), key.c_str());
      return false;
    }
  }
  return true;
}

double cart2comp(double angle_deg)
{
  double newangle = fmod(angle_deg - 450.0, 360.0);
  if (newangle < 0)
  {
    newangle += 360.0;
  }
  return newangle;
}

double comp2cart(double angle_deg)
{
  return fmod(450.0 + angle_deg, 360.0);
}

bool BHV_FormationCtrl::setFormation(std::string formation_type, std::string params)
{
  bool handled = false;
  formation_type = removeWhite(formation_type);
  params = removeWhite(params);
  if (m_formation_geomgr.isSupportedFormation(formation_type))
  {
    m_formation_type = formation_type;
    dbg_print("Formation type: %s\n", formation_type.c_str());
    handled = setParallelFormationType(formation_type, params);
  }
  else if (formation_type == "convoy")
  {
    dbg_print("Convoying is not supported yet\n");
    handled = false;
  }
  else
  {
    dbg_print("Invalid formation type: %s\n", formation_type.c_str());
    handled = false;
  }
  return handled;
}

//---------------------------------------------------------------
// Procedure: setParam()

/*
  TODO:
    [ ] Handle all desired parameters
    [ ] Include soft error handling where parameters are redundant and are not used for a specific formation
    [ ] Include a real time updates function where if FORMATION_UPDATES is published to, various utilities exist for incrementing or totally changing states during runtime

*/
bool BHV_FormationCtrl::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  m_debug = true;
  setDebug();

  bool handled = false;

  if ((param == "formation_type" || param == "formation"))
  {
    std::string formation_type = tolower(biteString(val, ':'));
    std::string formation_args = val;
    handled = setFormation(formation_type, val);
  }
  else if (param == "pos_idx")
  {
    val = removeWhite(val);
    if ((val == "auto" || (stoi(val) < 0) && isNumber(val)))
    {
      m_pos_idx = -1;
      handled = true;
    }
    else if (isNumber(val))
    {
      m_pos_idx = stoi(val);
      handled = true;
    }
  }
  else if (param == "capture_radius" && isNumber(val))
  {
    m_capture_radius = stod(val);
    handled = true;
  }
  else if (param == "slip_radius" && isNumber(val))
  {
    m_slip_radius = stod(val);
    handled = true;
  }
  else if (param == "sat_spd_lambda")
  {
    m_lambda_roa_spd_sr = stod(val);
    handled = true;
  }
  else if (param == "sat_spd_range")
  {
    m_rs_spd = stod(val);
    handled = true;
  }
  else if (param == "sat_hdg_lambda")
  {
    m_lambda_roa_hdg_sr = stod(val);
    handled = true;
  }
  else if (param == "sat_hdg_range")
  {
    m_rs_hdg = stod(val);
    handled = true;
  }
  else if (param == "kp_spd_1")
  {
    m_kp_spd_1 = stod(val);
    handled = true;
  }
  else if (param == "kp_spd_2")
  {
    m_kp_spd_2 = stod(val);
    handled = true;
  }
  else if (param == "kd_spd_1")
  {
    m_kd_spd_1 = stod(val);
    handled = true;
  }
  else if (param == "kd_spd_2")
  {
    m_kd_spd_2 = stod(val);
    handled = true;
  }
  else if (param == "kp_hdg_1")
  {
    m_kp_hdg_1 = stod(val);
    handled = true;
  }
  else if (param == "kp_hdg_2")
  {
    m_kp_hdg_2 = stod(val);
    handled = true;
  }
  else if (param == "kd_hdg_1")
  {
    m_kd_hdg_1 = stod(val);
    handled = true;
  }
  else if (param == "kd_hdg_2")
  {
    m_kd_hdg_2 = stod(val);
    handled = true;
  }
  else if (param == "kpc_spd")
  {
    m_kpc_spd = stod(val);
    handled = true;
  }
  else if (param == "kdc_spd")
  {
    m_kpc_spd = stod(val);
    handled = true;
  }
  else if (param == "kpc_hdg")
  {
    m_kpc_hdg = stod(val);
    handled = true;
  }
  else if (param == "kdc_hdg")
  {
    m_kdc_hdg = stod(val);
    handled = true;
  }
  else if (param == "max_speed")
  {
    m_max_speed = stod(val);
    handled = true;
  }
  else if (param == "min_queue_point_sep")
  {
    m_min_ctrl_point_sep = stod(val);
    handled = true;
  }
  else if (param == "max_queue_length")
  {
    m_max_ctrl_queue_length = stod(val);
    handled = true;
  }
  else if (param == "debug")
  {
    if (val == "true")
    {
      m_debug = true;
    }
    handled = true;
  }
  else if (param == "desired_formation_speed")
  {
    m_desired_speed = stod(val);
    handled = true;
  }
  else
  {
    handled = false;
  }
  dbg_print("Param: %s | Value: %s\n", param.c_str(), val.c_str());
  return handled;
}

void BHV_FormationCtrl::setDebug()
{
  m_debug = true;
  m_debug_fname = "debug_" + m_us_name + ".txt";
  m_formation_geomgr.setDebugFileName(m_debug_fname);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_FormationCtrl::onSetParamComplete()
{
  m_us_name = getOwnshipName();
  m_os_state.name = m_us_name;

  if (m_debug)
    setDebug();

  dbg_print("Idx: %d\n", m_pos_idx);

  m_ctrl_point_queue = CtrlPointQueue(m_min_ctrl_point_sep, m_max_ctrl_queue_length);

  dbg_print("Formation Type: %s\n", m_formation_type.c_str());
  // print all formation arguments
  for (auto arg : m_formation_params)
  {
    dbg_print("Formation Arg: %s: %s\n", arg.first.c_str(), arg.second.c_str());
  }
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_FormationCtrl::onHelmStart()
{
  updateMail();
  setInfoVars();
}

void BHV_FormationCtrl::setInfoVars()
{
  // Set the variables we want to subscribe to
  addInfoVars("NODE_REPORT_LOCAL");
  addInfoVars("AGENT_INFO");
  addInfoVars("LEAD_AGENT_INFO");
  addInfoVars("NODE_MESSAGE_LOCAL");
  addInfoVars("NAV_X");
  addInfoVars("NAV_Y");
  addInfoVars("NAV_HEADING");
  addInfoVars("NAV_SPEED");
  addInfoVars("FORMATION_IDX");
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_FormationCtrl::onIdleState()
{
  updateMail();
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_FormationCtrl::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_FormationCtrl::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_FormationCtrl::onIdleToRunState()
{
}

void BHV_FormationCtrl::assertInvariant(bool condition, const char *format, ...)
{
  if (!condition)
  {
    dbg_print("INVARIANT CONDITION FAILED\n");
    dbg_print(format);
    exit(1);
  }
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_FormationCtrl::onRunToIdleState()
{
}

void BHV_FormationCtrl::updateOwnshipState()
{
  // Here we will populate the m_ownship_state object with the information from the ownship, which is from a node record. We combine other information we have locally to populate the ownship state, such as the errors and everything else
  if (m_os_node_record.getName() != m_us_name || m_us_name == "larry" || m_os_node_record.getName() == "larry")
  {
    return;
  }
  double x, y, yaw, heading;
  double x_dot, y_dot, yaw_dot;
  double x_dotdot, y_dotdot, yaw_dotdot;
  double x_err, y_err, yaw_err;
  double x_dot_err, y_dot_err, yaw_dot_err;
  double x_dotdot_err, y_dotdot_err, yaw_dotdot_err;
  double u, v, r;
  double u_dot, v_dot, r_dot;
  double u_err, v_err, r_err;
  double u_dot_err, v_dot_err, r_dot_err;

  yaw = heading = 0;
  yaw_dot = 0;
  yaw_dotdot = 0;
  yaw_err = 0;
  yaw_dot_err = 0;
  yaw_dotdot_err = 0;

  u = v = r = 0;
  u_dot = v_dot = r_dot = 0;
  u_err = v_err = r_err = 0;
  u_dot_err = v_dot_err = r_dot_err = 0;

  x = m_os_node_record.getX();
  m_osx = x;
  y = m_os_node_record.getY();
  m_osy = y;
  heading = m_os_node_record.getHeading();
  m_osh = heading;

  yaw = comp2cart(heading);

  // TODO: numerically differentiate to get velocities and accelerations - not entirely necessary for the function of this app, but want to also track and investigate the performance

  // Populate the ownship state object

  m_os_state.time_se = m_os_node_record.getTimeStamp();
  m_os_state.name = m_os_node_record.getName();
  m_os_state.color = m_os_node_record.getColor();
  m_os_state.id = m_pos_idx;

  // Update all numerical derivatives
  double dt = m_os_state.time_se - m_os_state_prev.time_se;

  if (dt < 0.001)
  {
    dt = 0.001;
  }

  // Populate the ownship state object
  // World frame states for the agent
  m_os_state.q = {x, y, 0, 0, 0, yaw};
  // TODO: Include LPF filter for numerical differentiation to smooth
  arma::vec::fixed<6> q_dot_raw = (m_os_state.q - m_os_state_prev.q) / dt;

  // TODO: Make sure we are calculating all the errors and states we can be
  //  TODO: Do something with this alternative method of calculating q_dot - publish for future analysis
  u = m_os_node_record.getSpeed(); // Getting this directly from the node report

  double u_alt = q_dot_raw(0) * cos(yaw / RAD_TO_DEG) - q_dot_raw(1) * sin(yaw / RAD_TO_DEG); // Projecting the XY velocity into the body fixed frame - Not used

  v = q_dot_raw(0) * sin(yaw / RAD_TO_DEG) + q_dot_raw(1) * cos(yaw / RAD_TO_DEG); // Projecting the XY velocity into the body fixed frame

  m_os_state.q_dot = q_dot_raw;
  // arma::vec::fixed<6> q_dotdot = (m_os_state.q_dot - m_os_state_prev.q_dot)/dt;
  m_os_state.q_dotdot = {0, 0, 0, 0, 0, 0}; // Not used

  // Include the latest reference errors in this update
  m_os_state.r_reference = m_latest_r_reference;

  x_err = m_os_state.q(0) - m_desired_x_cp;
  y_err = m_os_state.q(1) - m_desired_y_cp;
  yaw_err = m_os_state.q(5) - m_desired_yaw_cp;

  m_os_state.q_err = {x_err, y_err, 0, 0, 0, yaw_err};

  x_dot_err = m_os_state.q_dot(0) - m_desired_x_dot_cp;
  y_dot_err = m_os_state.q_dot(1) - m_desired_y_dot_cp;
  yaw_dot_err = m_os_state.q_dot(5) - m_desired_yaw_dot_cp;

  m_os_state.q_dot_err = {x_dot_err, y_dot_err, 0, 0, 0, yaw_dot_err};

  m_os_state.q_dotdot_err = {x_dotdot_err, y_dotdot_err, 0, 0, 0, yaw_dotdot_err}; // Not used

  // Body fixed frame states
  // TODO: I think we can get these from a separate app which is running, otherwise we can make one
  r = q_dot_raw(5);

  m_os_state.nu = {u, v, 0, 0, 0, r};

  m_os_state.nu_dot = (m_os_state.nu - m_os_state_prev.nu) / dt;
  u_err = m_desired_surge_cp - u;
  v_err = 0 - v;
  r_err = m_desired_yaw_cp - r;
  m_os_state.nu_err = {u_err, v_err, 0, 0, 0, r_err};
  m_os_state.nu_dot_err = {u_dot_err, v_dot_err, 0, 0, 0, r_dot_err};

  m_agent_states[m_us_name] = m_os_state;
  m_agent_name_to_idx[m_us_name] = m_pos_idx;
  m_idx_to_agent_name[m_pos_idx] = m_us_name;

  m_os_state_prev = m_os_state;
}

void BHV_FormationCtrl::updateFormationGeometry()
{
  m_formation_geomgr.generateFormationGeometry(m_formation_type, m_num_agents, m_formation_params);
  m_ctrl_point_queue.clear();
  // print the representation
  dbg_print("Formation type: %s\n", m_formation_type.c_str());
  dbg_print("Num agents: %d\n", m_num_agents);
  dbg_print("Formation Geometry: %s\n", m_formation_geomgr.repr().c_str());
  dbg_print("Idx: %d\n", m_pos_idx);
}

void BHV_FormationCtrl::updateOwnshipNodeRecord()
{
  m_nr_t = getBufferTimeVal("NODE_REPORT_LOCAL");
  m_os_node_record = string2NodeRecord(getBufferStringVal("NODE_REPORT_LOCAL"));
  updateOwnshipState();
}

void BHV_FormationCtrl::updateLeadAgentStates()
{
  AgentStateInfo leader_info = AgentStateInfo(getBufferStringVal("LEAD_AGENT_INFO"));
  if (leader_info.name == "larry")
    return;
  m_agent_states[leader_info.name] = leader_info;
  m_agent_name_to_idx[leader_info.name] = leader_info.id;
  m_idx_to_agent_name[leader_info.id] = leader_info.name;
}

void BHV_FormationCtrl::updateAgentStates()
{
  AgentStateInfo agent_info = AgentStateInfo(getBufferStringVal("AGENT_INFO"));
  if (agent_info.name == "larry")
    return;
  m_agent_states[agent_info.name] = agent_info;
  m_agent_name_to_idx[agent_info.name] = agent_info.id;
  m_idx_to_agent_name[agent_info.id] = agent_info.name;
}

void BHV_FormationCtrl::updateMail()
{
  // Here, we capture all the data which is continuously streamed, as well as new data which helps transition states

  // We are receiving the instantaneous errors from each agent, as well as listening for a new point to include in our point queue, sent from the agent whom we are following

  // Update other agent states
  if (getBufferVarUpdated("AGENT_INFO"))
    updateAgentStates();

  if (getBufferVarUpdated("LEAD_AGENT_INFO"))
    updateLeadAgentStates();

  // print all the agent states we have

  if (getBufferVarUpdated("FORMATION_IDX"))
  {
    m_pos_idx = static_cast<int32_t>(getBufferDoubleVal("FORMATION_IDX"));
  }

  if (m_agent_states.size() > m_num_agents)
  {
    m_num_agents = m_agent_states.size();
    updateFormationGeometry();
  }

  if (getBufferVarUpdated("NODE_REPORT_LOCAL"))
    updateOwnshipNodeRecord();
}

void BHV_FormationCtrl::postQueueLead()
{
  m_prev_posting.set_active(false);
  postRepeatableMessage("VIEW_SEGLIST", m_prev_posting.get_spec());

  XYSegList queue = m_ctrl_point_queue.to_seglist();
  queue.set_color("points", m_os_state.color);
  queue.set_edge_color(m_os_state.color);
  queue.set_edge_color("");
  queue.set_vertex_color(m_os_state.color);
  queue.set_label(m_os_state.name + "_" + std::to_string(m_pos_idx));
  queue.set_label_color("white");
  queue.set_active(true);
  queue.set_id(to_string(m_pos_idx));
  queue.set_transparency(1);
  queue.set_edge_size(2);
  queue.set_vertex_size(10);
  postRepeatableMessage("VIEW_SEGLIST", queue.get_spec());
  m_prev_posting = queue;
}

void BHV_FormationCtrl::postUpdates()
{
  if (m_us_name == "larry")
  {
    // we don't know our name yet and shouldn't post an update
  }
  else
  {
    NodeMessage node_message;
    node_message.setSourceNode(m_us_name);
    node_message.setDestNode("all");

    if (m_pos_idx == 0)
    {
      node_message.setVarName(string("LEAD_AGENT_INFO"));
      postMessage("LEADER", "true");
    }
    else
    {
      node_message.setVarName(string("AGENT_INFO"));
    }

    node_message.setStringVal(m_os_state.repr());
    postMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
  }
}

IvPFunction *BHV_FormationCtrl::getSimpleSpeedPeak(double desired_speed)
{
  // Create the IvP function for speed
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  spd_zaic.setSummit(desired_speed);
  spd_zaic.setPeakWidth(0.5);
  spd_zaic.setBaseWidth(1.0);
  spd_zaic.setSummitDelta(0.8);

  // Extract the IvP function
  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  return spd_ipf;
}
IvPFunction *BHV_FormationCtrl::getSimpleHeadingPeak(double desired_heading)
{
  // Create the IvP function for heading
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(desired_heading);
  crs_zaic.setPeakWidth(0);
  crs_zaic.setBaseWidth(180.0);
  crs_zaic.setSummitDelta(0);
  crs_zaic.setValueWrap(true);

  // Extract the IvP function
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();
  return crs_ipf;
}

IvPFunction *BHV_FormationCtrl::getSimpleCoupledPeak(double desired_speed, double desired_heading)
{
  IvPFunction *crs_ipf = getSimpleHeadingPeak(desired_heading);
  IvPFunction *spd_ipf = getSimpleSpeedPeak(desired_speed);

  // Couple the heading and speed functions
  OF_Coupler coupler;
  IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);
  return ivp_function;
}

IvPFunction *BHV_FormationCtrl::followerConvoyBehavior()
{
  // TODO: If we are the first follower, we track the lead agents states to create the first CtrlPoint if it is beyond some minimum set distance. We then add this to our queue
  // We cruise towards the control point, and synchronize within the region of attraction
  // If we capture a point, and we are not the last follower, we propagate it to our follower

  if (m_formation_type == "convoy")
  {
    // Then we are in a serialized formation, and receiving embedded states/points propagated from the leader
    if (getBufferVarUpdated("CONVOY_POINT"))
      void();

    addInfoVars("CONVOY_POINT");
    /*
    include this point in the CtrlPointQueue
    */
  }

  return 0;
}

double clamp(double val, double min, double max)
{
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

double pmod(double a, double b)
{
  double r = fmod(a, b);
  if (r < 0)
  {
    r += b;
  }
  return r;
};

bool slipped(double dist, double slip_radius, double rel_ang)
{
  // We slip a point if the distance is within the slip radius, and is behind the agent
  // TODO: this is a bug at the moment
  return (dist < slip_radius && abs(rel_ang) > 45);
}

IvPFunction *BHV_FormationCtrl::followerFormationBehavior()
{
  // TODO: Add a point to the point queue if the new point would be beyond some minimum set distance

  // Get a new projected point off of the leader, add this point to the queue

  /*
    Add dbg prints after each line with me
  */

  // If we update the number of agents, we clear the queue
  double leader_heading = cart2comp(m_agent_states[m_idx_to_agent_name[0]].q[5]); // convert yaw to heading
  CtrlPoint leader_point = CtrlPoint(m_agent_states[m_idx_to_agent_name[0]]);
  /*
    We need to debug in the input and output of the getpoint function
  */

  dbg_print("Leader point: %s\n", leader_point.repr().c_str());
  dbg_print("Leader heading: %f\n", leader_heading);
  dbg_print("Pos idx: %d\n", m_pos_idx);
  CtrlPoint new_point = m_formation_geomgr.getPoint(leader_point, leader_heading, m_pos_idx);
  dbg_print("New point: %s\n", new_point.repr().c_str());
  m_ctrl_point_queue.addPoint(new_point);
  
  // Get the next desired point in the queue, which is not within our capture radius or slip radius
  assertInvariant(m_ctrl_point_queue.size() > 0, "CtrlPointQueue is empty\n");

  CtrlPoint desired_point;

  double dist_to_point = 0;
  double rel_ang = 0;
  do
  {
    // Observe the next point in the queue
    desired_point = m_ctrl_point_queue.peekPoint();

    // If we capture this point, or slip this point, we remove it from the queue
    dist_to_point = desired_point.dist(m_osx, m_osy);
    rel_ang = relAng(m_osx, m_osy, desired_point.x, desired_point.y)-m_osh;

    /*
      We are going to diagonose the slipped function, and its operation here
    */
    dbg_print("Point queue: %s\n", m_ctrl_point_queue.repr().c_str());
    dbg_print("Desired point: %s\n", desired_point.repr().c_str());
    dbg_print("Dist to point: %f | Slip radius: %f | Rel ang: %f\n", dist_to_point, m_slip_radius, rel_ang);
    dbg_print("Capture radius: %f\n", m_capture_radius);
    dbg_print("Slipped: %d\n", slipped(dist_to_point, m_slip_radius, rel_ang));

    if (dist_to_point < m_capture_radius || slipped(dist_to_point, m_slip_radius, rel_ang))
    {
      m_ctrl_point_queue.popPoint();
      // If we the point queue is empty, our desired point is going to be just the desired set point projected from the leader, and we'll break
      if (m_ctrl_point_queue.size() <= 1)
      {
        desired_point = new_point;
        break;
      }
    }
    else
    {
      // If we can track the next point in the queue without slipping or capturing it, we exit this loop, i.e. this point hasn't been captured, and has not been slipped
      break;
    }
    // Otherwise, we continue to inspect a qualifying point
  } while (true);

  postQueueLead();

  // Set the desired speed
  // Include the prescribed control laws - for heading, we could consider sat and inverse sat functions weighting how much we want to move towards a trackpoint vs. how much we want to copy the leader's heading
  /*
    The desired speed should encapsulate a smooth on-trajectory and off trajectory policy. For example,
  */  

  double sgn = (abs(rel_ang) > 90) && (dist_to_point < m_slip_radius) ? -1.0f : 1.0f;

  assertInvariant(m_ctrl_point_queue.size() > 0, "CtrlPointQueue is empty\n");
  m_os_state.r_err = m_ctrl_point_queue.getTrajectoryOdom(m_osx, m_osy)*sgn; // the error is from the current point, all the way up to the track point
  
  dbg_print("Reference error: %f\n", m_os_state.r_err);

  double reference_err = m_os_state.r_err;
  double desired_speed = desired_point.agent_state.nu[0]; // Leaders speed at this point in time
  m_desired_surge_cp = desired_speed;
  dbg_print("Desired speed: %f\n", desired_speed);
  dbg_print("Current speed: %f\n", m_os_state.nu[0]);
  double speed_err = desired_speed - m_os_state.nu[0];
  dbg_print("Speed error: %f\n", speed_err);
  double pd_terms = reference_err * (m_kp_spd_1 + m_kp_hdg_2 * abs(reference_err)) + speed_err * (m_kd_spd_1 + m_kd_spd_2 * abs(speed_err));

  // Get the sum of all other agent range and speed errors

  double r_err_sum = 0;
  double u_err_sum = 0;

  for (auto agent : m_agent_states)
  {
    if (agent.first == m_us_name)
      continue;
    r_err_sum += agent.second.r_err;
    u_err_sum += agent.second.nu_err[0];
  }
  double k_spd_sat = sat(m_rs_spd, m_lambda_roa_spd_sr, reference_err); // We only synchronize when we are close to our own desired state
  //double coupling_terms = k_spd_sat * (-m_kpc_spd * r_err_sum - m_kdc_spd * u_err_sum);
  double coupling_terms = 0;
  double ctrl_speed = desired_speed + pd_terms + coupling_terms;

  ctrl_speed = clamp(ctrl_speed, 0, m_max_speed);

  if (m_ctrl_point_queue.size() == 1)
  {
    // Then we just copy the leader speed
    ctrl_speed = desired_speed;
  }

  // TODO: slip condition is currently not working
  // TODO: Fix this with a better approach

  // Calculate the angle to the desired point
  double ang_to_point = relAng(m_osx, m_osy, desired_point.x, desired_point.y);

  double pd_hdg_terms = 0;

  double k_hdg_sat = sat(m_rs_hdg, m_lambda_roa_hdg_sr, reference_err);

  double coupling_hdg_terms = k_hdg_sat * 0;

  double ctrl_heading = ang_to_point + pd_hdg_terms + coupling_hdg_terms;

  dbg_print("Ang to point vs leader heading with reference error: %f vs %f, %0.2f\n", ang_to_point, leader_heading, reference_err);

  if (m_ctrl_point_queue.size() == 1)
  {
    // This is our track point which we are maintaining well, so we maintain the leader's heading, and accept that we may be close to, infront of, and behind this point, for some threshold
    ctrl_heading = leader_heading;
  }

  // Consider the slip radius for the capture condtion - if we just surpass our point we expect a small error but large relative angle - where we probably still just want to head straight

  return getSimpleCoupledPeak(ctrl_speed, ctrl_heading);
}

IvPFunction *BHV_FormationCtrl::followerIvPBehavior()
{
  if (m_formation_type == "convoy")
    return followerConvoyBehavior();
  else
    return followerFormationBehavior();
}

IvPFunction *BHV_FormationCtrl::leaderIvPBehavior()
{
  // TODO: Configure leader behavior rolls, i.e. synchronization, and maintenance of turning rates
  // for now we will only return a single peaked function for speed
  /*
    The leader's heading is determined by presumeably an external behavior, however the turning rate is damped by this behavior.
    For example, a waypoint behavior may want to steer the fleet quickly in a particular direction, the leader drives
    the heading of all the vehicles, but should do so in a way that all the vehicles can maintain a minimum distance
    error, while turning at the same rate as all the other vehicles. A formation will have a minimum and maximum turning
    radius, based on how quickly the vehicles can turn. If the fastest vehicle can only move at 1.4 m/s, which will be the
    speed of the vehicle away from the desired heading, along some optimal turning radius, the whole formation will have turning rate,
    bounded by this speed.

    coupling_terms = sum_of(all agent r_errs) + sum_of(all agent u_errs)
    m_desired_speed = u_n + coupling terms
    The lead agent only needs the coupling terms for compensating
  */

  double pd_terms = 0; // Leader defines the trajectory speed, zero errors - perfect - if not the best leader

  double r_err_sum = 0;
  double u_err_sum = 0;

  for (auto agent : m_agent_states)
  {
    if (agent.first == m_us_name)
      continue;
    r_err_sum += agent.second.r_err;
    u_err_sum += agent.second.nu_err[0];
  }

  double coupling_terms = -m_kpc_spd * r_err_sum - m_kdc_spd * u_err_sum;
  double ctrl_speed = m_desired_speed + pd_terms + coupling_terms;
  ctrl_speed = clamp(ctrl_speed, 0, m_max_speed);

  return getSimpleSpeedPeak(ctrl_speed);
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

/*
  TODO
   [ ] Include a point queue to allow changes between convoying and parallel formations
   [ ] Include synchronization speed policy
   [ ] With point queue, make the algorithm for using the ideal follow range and calculating the distance along an odometry while accounting for trailing poitns
   [ ] Have lead agent include synchronization based on ally errors
*/
IvPFunction *BHV_FormationCtrl::onRunState()
{
  // TODO: Have all agents publish their own error state and state derivative information, r_err, r_err_dot, heading_err, heading_err_dot, speed_err, speed_err_dot
  // TODO: Implement a SAT function which synchronizes agents when they are in the region of attraction
  // TODO: Add a slip radius to the tracking of a point - an agent should be able to be on the point and not oscillate around it

  // Consider all the new mail, and update the locally maintained states
  updateMail();

  // Produce an IvP Function depending on if we are a leader or a follower in the formation, for a leader centric formation approach
  IvPFunction *ivp_function = 0;

  if (m_pos_idx > 0 && m_pos_idx < m_num_agents)
  {
    ivp_function = followerIvPBehavior();
  }
  else if (m_pos_idx == 0)
  {
    ivp_function = leaderIvPBehavior();
  }
  else
  {
    ivp_function = 0;
  }

  // TODO: Handle the priority weight correctly

  double wgt = 200;
  if (ivp_function)
    ivp_function->setPWT(wgt);

  postUpdates();

  return ivp_function;
}
