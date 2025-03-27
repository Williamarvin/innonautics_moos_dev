
/* **************************************************************
  NAME: Raymond Turrisi
  ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
  FILE: BHV_ConvoyPD.cpp
  CIRC: November 2023

  LICENSE:
    This is unreleased BETA code. No permission is granted or
    implied to use, copy, modify, and distribute this software
    except by the author(s), or those designated by the author.
************************************************************** */

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include <cmath>
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "BuildUtils.h"
#include "BHV_ConvoyPD.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include <sstream>
#include "NodeRecord.h"
#include "NodeMessage.h" // In the lib_ufield library

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_ConvoyPD::BHV_ConvoyPD(IvPDomain domain) : IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "convoy_pd");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  m_desired_speed = 1.2;       // meters per second
  m_point_update_distance = 1; // meters
  m_ideal_follow_range = 7;    // meters
  m_full_stop_range = 3;       // meters
  m_capture_radius = 3;        // meters
  m_slip_radius = 6;           // meters
  m_max_speed = 1.5;           // meters per second
  m_coupled = false;

  // Ownship gains
  m_kp_spd_1 = 0.1, m_kp_spd_2 = 0, m_kp_spd_1 = 0.1, m_kp_spd_2 = 0, ki_spd = 0;
  m_kp_hdg_1 = 0.1, m_kp_hdg_2 = 0, m_kp_hdg_1 = 0.1, m_kp_hdg_2 = 0, ki_hdg = 0;

  // Coupling gains - Must be at least less than or equal to one half of the ownship gains, i.e. kp - 2 * kpc >= 0
  m_kpc_spd = 0.0;
  m_kdc_spd = 0.0;

  m_is_leader = false;
  m_is_midship = false;
  m_is_tail = false;
  m_contact = "NULL";
  m_follower = "NULL";
  m_contact_list_str = "";
  m_has_broadcast_contact = false;

  m_osx = 0;
  m_osy = 0;
  m_osh = 0;
  m_osx_prv = 0;
  m_osy_prv = 0;
  m_osh_prv = 0;
  m_interval_odo = 0;    // meters
  m_eps = 0.01;          // Meters - sufficient errors
  m_posted_points = 0;   // counter
  m_max_lag_length = 20; // maximum tail length to be left behind - we have this so the leaders trajectory will mostly be followed
  m_color = "yellow";
  m_redudant_update_interval = 2; // by default, every two seconds, we will deliberately post a redundant message to our neighbors
  m_dist_err = 0;
  m_spd_err = 0;

  m_debug = true;

  m_debug_fname = "debug_" + m_us_name + ".txt";

  m_ownship = XYPoint(m_osx, m_osy);
  m_target = XYPoint(m_osx, m_osy);

  m_cpq = ConvoyPointQueue();

  m_nav_x_k = "NAV_X";
  m_nav_y_k = "NAV_Y";
  m_nav_h_k = "NAV_HEADING";
  m_nav_spd_k = "NAV_SPEED";
  m_leader_k = "LEADER";
  m_contact_k = "CONTACT";
  m_agent_info_k = "AGENT_INFO*";
  m_lead_point_k = "NEW_LEAD_POINT";
  m_contact_list_k = "CONTACTS_LIST";
  m_task_state_k = "TASK_STATE";
  m_whotowho_k = "A_FOLLOWING_B";
  m_nrl_k = "NODE_REPORT_LOCAL";
  m_updates_var_k = "CONVOY_UPDATES";
  m_ext_ordering_k = "EXT_ORDERING";

  string infovars = m_nav_h_k +
                    "," + m_nav_y_k +
                    "," + m_nav_h_k +
                    "," + m_nav_spd_k +
                    "," + m_leader_k +
                    "," + m_contact_k +
                    "," + m_agent_info_k +
                    "," + m_nrl_k +
                    "," + m_lead_point_k +
                    "," + m_contact_list_k +
                    "," + m_task_state_k +
                    "," + m_whotowho_k +
                    "," + m_ext_ordering_k +
                    "," + m_updates_var_k;

  addInfoVars(infovars);
}

//---------------------------------------------------------
// Procedure: dbg_print()
bool BHV_ConvoyPD::dbg_print(const char *format, ...)
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

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_ConvoyPD::setParam(string param, string val)
{

  param = tolower(param);

  if ((param == "is_leader") && isBoolean(val))
  {
    return setBooleanOnString(m_is_leader, val);
  }
  else if (param == "desired_speed" && isNumber(val))
  {
    m_desired_speed = stod(val);
    return true;
  }
  else if (param == "capture_radius" && isNumber(val))
  {
    m_capture_radius = stod(val);
    return true;
  }
  else if (param == "updates")
  {
    m_updates_var_k = val;
    addInfoVars(m_updates_var_k);
    return true;
  }
  else if (param == "slip_radius" && isNumber(val))
  {
    m_slip_radius = stod(val);
    return true;
  }
  else if (param == "ideal_follow_range" && isNumber(val))
  {
    m_ideal_follow_range = stod(val);
    return true;
  }
  else if (param == "full_stop_range" && isNumber(val))
  {
    m_full_stop_range = stod(val);
    return true;
  }
  else if (param == "max_lag_length" && isNumber(val))
  {
    m_max_lag_length = stod(val);
    return true;
  }
  else if (param == "max_speed" && isNumber(val))
  {
    m_max_speed = stod(val);
    return true;
  }
  else if (param == "type")
  {
    m_type_assignment = val;
    return true;
  }
  else if (param == "point_update_distance" && isNumber(val))
  {
    m_point_update_distance = stod(val);
    return true;
  }
  else if (param == "kp_spd_1" && isNumber(val))
  {
    m_kp_spd_1 = stod(val);
    return true;
  }
  else if (param == "kp_spd_2" && isNumber(val))
  {
    m_kp_spd_2 = stod(val);
    return true;
  }
  else if (param == "kd_spd_1" && isNumber(val))
  {
    m_kd_spd_1 = stod(val);
    return true;
  }
  else if (param == "kd_spd_2" && isNumber(val))
  {
    m_kd_spd_2 = stod(val);
    return true;
  }
  else if (param == "ki_spd" && isNumber(val))
  {
    ki_spd = stod(val);
    return true;
  }
  else if (param == "kp_hdg_1" && isNumber(val))
  {
    m_kp_hdg_1 = stod(val);
    return true;
  }
  else if (param == "kp_hdg_2" && isNumber(val))
  {
    m_kp_hdg_2 = stod(val);
    return true;
  }
  else if (param == "kd_hdg_1" && isNumber(val))
  {
    m_kd_hdg_1 = stod(val);
    return true;
  }
  else if (param == "kd_hdg_2" && isNumber(val))
  {
    m_kd_hdg_2 = stod(val);
    return true;
  }
  else if (param == "ki_hdg" && isNumber(val))
  {
    ki_hdg = stod(val);
    return true;
  }
  // Coupling gains
  else if (param == "coupled" && isBoolean(val))
  {
    m_coupled = tolower(val) == "true" ? true : false;
    return true;
  }
  else if (param == "kpc_spd" && isNumber(val))
  {
    m_kpc_spd = stod(val);
    return true;
  }
  else if (param == "kdc_spd" && isNumber(val))
  {
    m_kdc_spd = stod(val);
    return true;
  }

  // If not handled above, then just return false;
  return (false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_ConvoyPD::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_ConvoyPD::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_ConvoyPD::onIdleState()
{
  updateMessages();
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_ConvoyPD::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_ConvoyPD::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_ConvoyPD::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_ConvoyPD::onRunToIdleState()
{
}

void BHV_ConvoyPD::clamp(double &value, double min, double max)
{
  if (value < min)
  {
    value = min;
  }
  if (value > max)
  {
    value = max;
  }
}

double BHV_ConvoyPD::pmod(double a, double b)
{
  double r = fmod(a, b);
  if (r < 0)
  {
    r += b;
  }
  return r;
};

bool BHV_ConvoyPD::shouldRepost(std::string field)
{
  if (m_next_redundant_update.count(field) == 0)
  {
    m_next_redundant_update[field] = 0;
  }
  return m_next_redundant_update[field] < m_latest_buffer_time;
}

void BHV_ConvoyPD::scheduleRepost(std::string field)
{
  m_next_redundant_update[field] = m_latest_buffer_time + m_redudant_update_interval;
}

//---------------------------------------------------------------
// Procedure: postStateMessages()
//   Purpose: Invoked when idle and when running publish generally maintained state variables

void BHV_ConvoyPD::postStateMessages()
{
  postAgentInfo();
  generalBroadcasts();
}

void BHV_ConvoyPD::postAgentInfo()
{
  NodeMessage node_message;

  node_message.setSourceNode(m_us_name);
  node_message.setDestNode("all");
  node_message.setVarName(string("AGENT_INFO_") + toupper(m_us_name));

  node_message.setStringVal(m_self_agent_info.repr());
  postRepeatableMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
}

void BHV_ConvoyPD::postDistError(double err)
{

  m_prev_err_point.set_active(false);

  postMessage("VIEW_POINT", m_prev_err_point.get_spec());
  XYPoint diff_msg(m_osx, m_osy + 4);
  diff_msg.set_label("dist_err=" + floatToString(err, 3));
  diff_msg.set_vertex_color("invisible");
  postMessage("VIEW_POINT", diff_msg.get_spec());
  m_prev_err_point = diff_msg;
}

void BHV_ConvoyPD::postLeadership()
{
  {
    m_contact = "*";
    m_follower_to_leader_mapping[m_us_name] = "*";
    NodeMessage node_message;
    node_message.setSourceNode(m_us_name);
    node_message.setDestNode("all");
    node_message.setVarName(m_whotowho_k);
    node_message.setStringVal(tolower(m_us_name) + "_following_" + "*");
    postRepeatableMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
    scheduleRepost("leader_broadcast");
  }
}

void BHV_ConvoyPD::postOrdering()
{
  // Contacts list is n-1 agents since it does not include the ownship

  // When we have a mapping of all known agents, we generate our ordering, and share it to all other nodes

  std::map<string, string>::iterator it = m_follower_to_leader_mapping.begin();

  for (; it != m_follower_to_leader_mapping.end(); ++it)
  {
    m_leader_to_follower_mapping[it->second] = it->first;
  }

  std::string ahead = "*";
  std::string tail = "*";
  m_ordering_str.clear();
  m_ordering_vector.clear();
  for (int i = 0; i < m_leader_to_follower_mapping.size(); i++)
  {
    tail = m_leader_to_follower_mapping[ahead];
    m_ordering_vector.push_back(tail);
    ahead = tail;
    if (m_us_name == tail)
    {
      m_place_in_convoy = i;
    }
  }

  for (int i = 0; i < m_ordering_vector.size(); i++)
  {
    m_ordering_str += m_ordering_vector[i];
    if (i + 1 < m_ordering_vector.size())
    {
      m_ordering_str += ",";
    }
  }

  if (tail == m_us_name)
  {
    m_is_tail = true;
  }
  else
  {
    m_is_tail = false;
  }
  if (true)
  {
    m_follower = m_leader_to_follower_mapping[m_us_name];
  }

  else if (!m_is_leader && !m_is_tail)
  {
    m_is_midship = true;
  }

  NodeMessage node_message;
  node_message.setSourceNode(m_us_name);
  node_message.setDestNode("all");
  node_message.setVarName(m_ext_ordering_k);
  node_message.setStringVal(m_ordering_str);
  postRepeatableMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());

  // dbg_print("ordering: %s\n", m_ordering_str.c_str());
  postRepeatableMessage("ORDERING", m_ordering_str);
}

void BHV_ConvoyPD::propagatePoint(ConvoyPoint prv_cp)
{
  // Send the encoded convoy point as a node message to our follower
  NodeMessage node_message;
  node_message.setSourceNode(m_us_name);
  node_message.setDestNode(m_follower);
  node_message.setVarName(m_lead_point_k);
  node_message.setStringVal(prv_cp.repr());
  postRepeatableMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
}

void BHV_ConvoyPD::generalBroadcasts()
{
  if (m_is_leader)
  {
    if (shouldRepost("leader_broadcast"))
    {
      postLeadership();
      scheduleRepost("leader_broadcast");
    }
  }

  if (shouldRepost("ordering"))
  {
    postOrdering();
    scheduleRepost("ordering");
  }
}

void BHV_ConvoyPD::seedPoints()
{
  if ((abs(m_osx - m_osx_prv) > m_eps) &&
      (abs(m_osy - m_osy_prv) > m_eps))
  {
    double dx = m_osx - m_osx_prv;
    double dy = m_osy - m_osy_prv;
    double dh = m_osh - m_osh_prv;

    m_osx_prv = m_osx;
    m_osy_prv = m_osy;
    m_osh_prv = m_osh;
    m_interval_odo += sqrt(dy * dy + dx * dx);
  }
  if (m_interval_odo >= m_point_update_distance)
  {

    XYPoint cp(m_osx, m_osy);
    // cp.set_duration(20);
    cp.set_vertex_color("red");
    cp.set_vertex_size(10);
    cp.set_active(true);
    cp.set_label(to_string(m_posted_points));
    cp.set_label_color("invisible");
    cp.set_id(to_string(m_posted_points));
    // postRepeatableMessage("VIEW_POINT", cp.get_spec());
    ConvoyPoint cpp(cp);

    cpp.set_st(getBufferCurrTime());
    cpp.set_spd(m_speed);
    cpp.set_lh(m_osh);
    cpp.set_lhr(m_osh_dot);

    // m_cpq.add_point(cpp);
    ConvoyPoint cp_alt(cpp.repr()); // temporary

    m_interval_odo = 0;
    double dist_between = m_cpq.get_dist_to_target();

    NodeMessage node_message;
    node_message.setSourceNode(m_us_name);
    node_message.setDestNode(m_follower);
    node_message.setVarName(m_lead_point_k);
    node_message.setStringVal(cpp.repr());
    postRepeatableMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
    m_posted_points++;
  }
}

//---------------------------------------------------------------
// Procedure: updateMessages()
//   Purpose: Invoked when idle and when running to keep variables up to date

void BHV_ConvoyPD::updateMessages()
{
  // file = fopen(fname.c_str(), "a");

  // If update our own state variables on this iteration pulling from the latest information in the queue
  updateOwnshipState();

  // Check to see if we are the leader
  if (getBufferVarUpdated(m_leader_k))
    updateIsLeader();

  // If we recieved a new list of contacts, we update our list
  if (getBufferVarUpdated(m_contact_list_k))
    updateContactList();

  // If we received an update to our task state bid, we know who we are following
  if (getBufferVarUpdated(m_task_state_k))
    updateCheckForContact();

  // If we receive a new lead point from whom we are follwoing, we add this to our queue
  if (getBufferVarUpdated(m_lead_point_k))
    updateLeadPoint();

  // If we use the local node report to get our current color
  if (getBufferVarUpdated(m_nrl_k))
    handleNodeReport();

  // We check to see if we received another node message where someone broadcasted who they are following, to update our follower to leader and leader to follower mapping
  if (getBufferVarUpdated(m_whotowho_k))
    updateFtoLMapping();

  // Ourselves and all other agents publish their known ordering, since it is possible some or all agents are missing some information or updates. If we recieve information which conveys we are missing information, we definitely are, so we simply update our ordering with this information (redundant but a fail safe)
  if (getBufferVarUpdated(m_ext_ordering_k))
    updateExtOrdering();

  // If we received optional updates to our choice of parameters and state
  if (getBufferVarUpdated(m_updates_var_k))
    updateUpdatesVar();

  // Ourselves and all other agents distribute our own agent info, for some local useage in ConvoyPD, but later to mostly be used for synchronization
  for (int i = 0; i < m_contact_list.size(); i++)
  {
    if (getBufferVarUpdated("AGENT_INFO_" + toupper(m_contact_list[i])))
    {
      updateAgentInfo(m_contact_list[i]);
    }
  }
}

void BHV_ConvoyPD::updateAgentInfo(std::string name)
{
  string msg = getBufferStringVal("AGENT_INFO_" + toupper(name));

  m_contacts_lookup[name] = AgentInfo(msg);
  map<string, AgentInfo>::iterator it = m_contacts_lookup.begin();

  if (m_contacts_lookup.count(m_contact))
  {
    AgentInfo cn = m_contacts_lookup[m_contact];
    m_target.set_vx(cn.x);
    m_target.set_vy(cn.y);
  }
}

void BHV_ConvoyPD::updateOwnshipState()
{
  // Not to be continuously reset, but the name wasnt available after setting parameters
  m_debug_fname = "debug_" + m_us_name + ".txt";

  // Update our known state information with the latest buffer values
  m_latest_buffer_time = getBufferCurrTime();
  double dt = (m_latest_buffer_time - m_osh_tprv);
  m_osh_tprv = m_latest_buffer_time;

  m_osx = getBufferDoubleVal(m_nav_x_k);
  m_osy = getBufferDoubleVal(m_nav_y_k);
  m_osh = getBufferDoubleVal(m_nav_h_k);
  m_speed = getBufferDoubleVal(m_nav_spd_k);

  // Change in heading +- 180 degrees always
  double dh = pmod(m_osh - m_osh_prv + 180, 360) - 180;

  // Incorporate an exponential moving average filter
  double m_osh_dot_instant = dh / dt;
  double alpha = 0.1;
  double ff = exp(-alpha * dt);
  m_osh_dot = m_osh_dot * ff + m_osh_dot_instant * (1 - ff);
  m_osh_prv = m_osh;

  m_ownship.set_vx(m_osx);
  m_ownship.set_vy(m_osy);

  // TODO: Should find a better place for this and be more consistent with our usage
  m_self_agent_info.name = m_us_name;
  m_self_agent_info.x = m_osx;
  // x_dot - tbd
  m_self_agent_info.y = m_osy;
  // y_dot - tbd
  m_self_agent_info.h = m_osh;
  m_self_agent_info.h_dot = m_osh_dot;
  m_self_agent_info.u = m_speed;
  // v - tbd
  m_self_agent_info.utc = m_latest_buffer_time;
  m_self_agent_info.color = m_color;
  m_self_agent_info.dist_err = m_dist_err;
  m_self_agent_info.spd_err = m_spd_err;
}

void BHV_ConvoyPD::updateCapturePoint()
{

  // If we are not the leader and we have points to capture
  if (!m_is_leader && m_cpq.size() > 0)
  {

    XYPoint next_point = m_cpq.front().p;
    double dy = next_point.get_vy() - m_osy;
    double dx = next_point.get_vx() - m_osx;
    double dist = hypot(dy, dx);
    double ang_to_trg_rads = atan2(dy, dx);
    double phi = ang_to_trg_rads - headingToRadians(m_osh);

    phi = pmod(phi + M_PI, 2 * M_PI) - M_PI;

    // If our distance to the target is less than our capture radius, then we captured it
    // If the point is essentially behind us but close enough to cause problems, we let is slip
    // i.e. The point is > +- 90 degrees off our current heading, and may cause inconveniences, so we don't need to consider it and should discard it
    if (dist < m_capture_radius || (dist < m_slip_radius && abs(phi) > M_PI / 2))
    {
      ConvoyPoint capture_point_cp = m_cpq.dequeue();
      XYPoint capture_point = capture_point_cp.p;
      capture_point.set_active(false);
      // We remove the view point from pMarineViewer
      postRepeatableMessage("VIEW_POINT", capture_point.get_spec());

      // If we are not the tail agent, then we can propagate this point to our follower
      if (!m_is_tail)
      {
        propagatePoint(capture_point_cp);
      }
    }
  }
}

void BHV_ConvoyPD::updateIsLeader()
{
  // If we published the leader mode in a successful convoy bid, we denote this
  m_is_leader = tolower(getBufferStringVal(m_leader_k)) == "true" ? true : false;
}

void BHV_ConvoyPD::updateExtOrdering()
{
  std::string ext_ordering = getBufferStringVal(m_ext_ordering_k);

  if (ext_ordering.size() > m_ordering_str.size())
  {
    vector<string> ext_order_vector = parseString(ext_ordering, ',');
    for (int i = 0; i < ext_order_vector.size() - 1; i++)
    {

      if (ext_order_vector[i] != "" && ext_order_vector[i + 1] != "")
      {
        string l = ext_order_vector[i];
        string f = ext_order_vector[i + 1];
        // REM: this was a better place to re-register for known agents other than the contacts list - need to look into this when making this code more lean
        addInfoVars("AGENT_INFO_" + toupper(f));
        addInfoVars("AGENT_INFO_" + toupper(l));
        m_follower_to_leader_mapping[f] = l;
        if (m_follower_to_leader_mapping[m_us_name] != m_contact)
        {
          m_contact = m_follower_to_leader_mapping[m_us_name];
        }
      }
      else
      {
        break;
      }
    }
  }
}

void BHV_ConvoyPD::updateContactList()
{

  // We received our total contact list, and update our list of contacts to consider
  m_contact_list_str = getBufferStringVal(m_contact_list_k);
  m_contact_list = parseString(m_contact_list_str, ",");
}

void BHV_ConvoyPD::updateCheckForContact()
{
  // We use the task state to see if we won a convoy bid, if we have, then we broadcast who we are following for our other contacts
  m_task_state = tolower(getBufferStringVal(m_task_state_k));

  // If we won the convoy bid, we know who we are trailing
  size_t idx1 = m_task_state.find("id=follow_");
  size_t idx2 = m_task_state.find("bidwon");

  if ((idx1 != std::string::npos) && (idx2 != std::string::npos) && !m_has_broadcast_contact && !m_is_leader)
  {
    m_contact = m_task_state.substr(idx1 + 10);
    m_contact = biteString(m_contact, ',');
    m_follower_to_leader_mapping[m_us_name] = m_contact;
    NodeMessage node_message;
    node_message.setSourceNode(m_us_name);
    node_message.setDestNode("all");
    node_message.setVarName(m_whotowho_k);
    node_message.setStringVal(tolower(m_us_name) + "_following_" + tolower(m_contact));
    postRepeatableMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
    m_has_broadcast_contact = true;
  }
}

void BHV_ConvoyPD::updateLeadPoint()
{

  std::string msg = getBufferStringVal(m_lead_point_k);
  m_cpq.add_point(ConvoyPoint(msg));
  XYPoint cp = m_cpq.m_points.back().p;
  cp.set_vertex_color(m_color);
  postRepeatableMessage("VIEW_POINT", cp.get_spec());
}

void BHV_ConvoyPD::updateFtoLMapping()
{
  std::string atob_msg = getBufferStringVal(m_whotowho_k);
  string agent_a = biteString(atob_msg, '_');
  biteString(atob_msg, '_');
  string agent_b = atob_msg;
  addInfoVars("AGENT_INFO_" + toupper(agent_a));
  addInfoVars("AGENT_INFO_" + toupper(agent_b));

  m_follower_to_leader_mapping[agent_a] = agent_b;
  m_has_broadcast_leadership = false;

  std::map<string, string>::iterator it = m_follower_to_leader_mapping.begin();
}

void BHV_ConvoyPD::updateUpdatesVar()
{
  string updates_msg = getBufferStringVal(m_updates_var_k);
  m_updates_buffer = updates_msg;
  vector<string> updates = parseQuotedString(updates_msg, ',');

  vector<string>::iterator it = updates.begin();

  string coupled_k = "coupled";
  string spd_up_k = "spd_up";
  string spd_dwn_k = "spd_dwn";
  string idf_up_k = "idf_up";
  string idf_dwn_k = "idf_dwn";

  for (; it != updates.end(); ++it)
  {
    // a segment may equal gains='kp1,kd1,ki1'
    string segment = *it;
    // gains

    string param = biteString(segment, '=');
    //'kp1,kd1,ki1';
    string value = segment;

    if (param == m_des_spd_k)
    {
      m_desired_speed = stod(value);
    }
    else if (param == m_point_update_dist_k)
    {
      m_point_update_distance = stod(value);
    }
    else if (param == spd_up_k)
    {
      double val = stod(value);
      clamp(val, 0, m_max_speed);
      m_desired_speed += val;
    }
    else if (param == spd_dwn_k)
    {
      double val = stod(value);
      clamp(val, 0, m_max_speed);
      m_desired_speed -= val;
    }
    else if (param == idf_up_k)
    {
      double val = stod(value);
      clamp(val, 0.1, 1000);
      m_ideal_follow_range += val;
    }
    else if (param == idf_dwn_k)
    {
      double val = stod(value);
      clamp(val, 0.1, 1000);
      m_ideal_follow_range -= stod(value);
    }
    else if (param == coupled_k)
    {
      m_coupled = tolower(value) == "true" ? true : false;
      dbg_print("received update: coupled %s\n", m_coupled == true ? "true" : "false");
    }
  }
}

void BHV_ConvoyPD::handleNodeReport()
{

  string nr_spec = getBufferStringVal(m_nrl_k);
  vector<string> fields = parseQuotedString(nr_spec, ',');
  for (vector<string>::iterator it = fields.begin(); it != fields.end(); ++it)
  {
    string entry = *it;
    string param = biteString(entry, '=');
    string val = entry;
    if (tolower(param) == "color")
    {
      m_color = val;
    }
  }
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction *BHV_ConvoyPD::onRunState()
{
  // Part 1: Build the IvP function
  updateMessages();
  postStateMessages();

  IvPFunction *ivp_function;

  // If we are the leader, we don't do any of the driving for this convoypd behavior, - it returns a weight of 0, but is a placeholder here for synchronization
  if (m_is_leader)
  {
    seedPoints();
    ZAIC_PEAK spd_zaic(m_domain, "speed");

    bool has_tail = m_leader_to_follower_mapping[m_us_name] != "" || m_leader_to_follower_mapping[m_us_name] != "NULL";
    double set_spd = m_desired_speed;
    if (m_coupled && has_tail)
    {
      double tail_dist_err = m_contacts_lookup[m_leader_to_follower_mapping[m_us_name]].dist_err;
      double tail_spd_err = m_contacts_lookup[m_leader_to_follower_mapping[m_us_name]].spd_err;
      double tail_comp = 0.5 * (m_kpc_spd * tail_dist_err + m_kdc_spd * tail_spd_err);
      dbg_print("Is leader: tail comp %0.2f\n", tail_comp);
      set_spd -= tail_comp;
    }
    clamp(set_spd, 0, m_max_speed);
    spd_zaic.setSummit(set_spd);
    spd_zaic.setPeakWidth(0.25);
    spd_zaic.setBaseWidth(0.5);
    spd_zaic.setSummitDelta(0.5);

    if (spd_zaic.stateOK() == false)
    {
      string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
      postWMessage(warnings);
      return (0);
    }

    ivp_function = spd_zaic.extractIvPFunction();
  }
  else
  {
    // If we are a follower, we have some work to do

    // Check to see if we captured our current point if we have a target, if we did, execute a procedure
    updateCapturePoint();

    // We build a speed function, and our speed policy is governed by a PD controller
    ZAIC_PEAK spd_zaic(m_domain, "speed");

    double dist_to_target;

    // Obtain our total  distance to the target, and if our total distance is passed our max lead length, we drop and prioritize points to catchup
    ConvoyPoint next_convoy_point;
    do
    {
      // Only the length of the segments formed by all the points in queue
      dist_to_target = m_cpq.get_dist_to_target();
      if (m_cpq.size() != 0)
      {
        dist_to_target += m_cpq.front().dist(m_ownship);
        dist_to_target += m_cpq.back().dist(m_target);
      }
      else
      {
        dist_to_target += distPointToPoint(m_target, m_ownship);
      }

      // If the distance exceeds our max lead length, we cut the lead to the max length
      if (dist_to_target > (m_ideal_follow_range + m_max_lag_length) && m_cpq.size() > 0)
      {
        next_convoy_point = m_cpq.dequeue();
        XYPoint prv_point = next_convoy_point.p;
        prv_point.set_active(false);
        postRepeatableMessage("VIEW_POINT", prv_point.get_spec());
      }
    } while (dist_to_target > (m_ideal_follow_range + m_max_lag_length) && m_cpq.size() > 0);

    // Obtain our next target - a point in the queue or our target itself
    if (m_cpq.size() != 0)
    {
      next_convoy_point = m_cpq.front();
    }
    else
    {
      next_convoy_point = ConvoyPoint(m_target);
    }

    double leader_speed;

    // Obtain the encoced leader speed of the lead point - if there are no points in this queue, we maintain our desired speed
    if (m_cpq.size() != 0)
    {
      leader_speed = next_convoy_point.leader_speed;
    }
    else
    {
      leader_speed = m_desired_speed;
    }
    // Compute the error in desired follow range, and the actual range to target
    m_dist_err = dist_to_target - m_ideal_follow_range;

    // Compute the error in our encoded speed
    m_spd_err = leader_speed - m_speed;

    // Generate the set speed with the ownship errors and gains only
    double set_spd = m_desired_speed + (m_kp_spd_1 + m_kp_spd_2 * abs(m_dist_err)) * (m_dist_err) + (m_kd_spd_1 + m_kd_spd_2 * abs(m_spd_err)) * (m_spd_err);

    // If we are coupled, then add on the coupling terms
    if (m_coupled)
    {
      double tail_dist_err, tail_spd_err, trail_dist_err, trail_spd_err;
      bool has_trail = m_follower_to_leader_mapping[m_us_name] != "" || m_follower_to_leader_mapping[m_us_name] != "NULL";
      bool has_tail = m_leader_to_follower_mapping[m_us_name] != "" || m_leader_to_follower_mapping[m_us_name] != "NULL";

      if (has_tail)
      {
        tail_dist_err = m_contacts_lookup[m_leader_to_follower_mapping[m_us_name]].dist_err;
        tail_spd_err = m_contacts_lookup[m_leader_to_follower_mapping[m_us_name]].spd_err;
      }
      else
      {
        tail_dist_err = 0;
        tail_spd_err = 0;
      }
      if (has_trail)
      {
        trail_dist_err = m_contacts_lookup[m_follower_to_leader_mapping[m_us_name]].dist_err;
        trail_spd_err = m_contacts_lookup[m_follower_to_leader_mapping[m_us_name]].spd_err;
      }
      else
      {
        trail_dist_err = 0;
        trail_spd_err = 0;
      }
      
      double trail_comp = m_kpc_spd * trail_dist_err + m_kdc_spd * trail_spd_err;
      double tail_comp = m_kpc_spd * tail_dist_err + m_kdc_spd * tail_spd_err;
      trail_comp = 0;
      dbg_print("Not leader or tail: trail comp %0.2f, tail comp: %0.2f\n", trail_comp, tail_comp);
      set_spd -= (trail_comp + tail_comp);
    }
    
    dbg_print("Set spd before clamp %0.2f\n", set_spd);

    // Clamp the result to be between zero and the max speed (assume no reverse thrust)
    clamp(set_spd, 0, m_max_speed);

    // If we are too close to the target, we simply halt
    if (dist_to_target <= m_full_stop_range)
      set_spd = 0;

    postDistError(m_dist_err);

    spd_zaic.setSummit(set_spd);
    spd_zaic.setPeakWidth(0.5);
    spd_zaic.setBaseWidth(1.0);
    spd_zaic.setSummitDelta(0.8);

    if (spd_zaic.stateOK() == false)
    {
      string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
      postWMessage(warnings);
      return (0);
    }

    // Now that we have our angle to to our target, which is either a waypoint or our contact, we now incorporate a PD controller for trajectory tracking
    double ang_to_target = relAng(m_osx, m_osy, next_convoy_point.p.get_vx(), next_convoy_point.p.get_vy());
    // We have our current heading, we have the angle to the target, and we have the heading which was dropped by the leader. The heading dropped by the leader is the "trajectory" with an optimal turning rate

    // TODO: This currently does nothing - we only only have a PD controller for the speed policy
    double hdg_err, hdg_err_dot;
    if (m_cpq.size() != 0)
    {
      hdg_err = ((next_convoy_point.leader_heading + ang_to_target) / 2) - m_osh;
      hdg_err_dot = (next_convoy_point.leader_heading_rate - m_osh_dot);
    }
    else
    {
      hdg_err = ang_to_target - m_osh;
      hdg_err_dot = 0;
    }

    // TODO: Not used yet
    double desired_course = ang_to_target + (m_kd_hdg_1 + m_kd_hdg_2 * abs(hdg_err_dot)) * (hdg_err_dot);

    ZAIC_PEAK crs_zaic(m_domain, "course");
    crs_zaic.setSummit(ang_to_target);
    crs_zaic.setPeakWidth(0);
    crs_zaic.setBaseWidth(180.0);
    crs_zaic.setSummitDelta(0);
    crs_zaic.setValueWrap(true);
    if (crs_zaic.stateOK() == false)
    {
      string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
      postWMessage(warnings);
      return (0);
    }

    if (ivp_function && !m_is_leader && (m_cpq.size() == 0))
    {
      spd_zaic.setSummit(m_desired_speed*0.5);
    }

    IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
    IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();

    OF_Coupler coupler;
    ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);
  }

  double wgt = 200;

  if (ivp_function)
    ivp_function->setPWT(wgt);
  

  return (ivp_function);
}
