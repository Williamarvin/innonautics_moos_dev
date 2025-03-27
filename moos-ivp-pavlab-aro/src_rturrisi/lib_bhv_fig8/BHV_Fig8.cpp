/************************************************************/
/*    NAME: Raymond Turrisi                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Fig8.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_Fig8.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "NodeRecordUtils.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include "XYPolygon.h"
#include "AngleUtils.h"
#include "NodeMessage.h" // In the lib_ufield library

using namespace std;

/*

  We assign parameters to define a lemniscate curve
    center x/y
    width/height
    direction
    total number of points
    alpha (angle of the curve)
  An agent starts at i=0, and moves along the curve up to the number of desired points, and then it resets to i=0

  We can send updates to UPDATES_LEMNISCATE, which change the parameters in real time

  Other parameters:
    capture radius
    slip radius
    desired speed


*/
//---------------------------------------------------------------
// Constructor

BHV_Fig8::BHV_Fig8(IvPDomain domain) : IvPBehavior(domain), m_lemniscate_calculator()
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "fig8");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  BHV_Fig8::initStateVars();
}

bool BHV_Fig8::dbg_print(const char *format, ...)
{
  if (p_debug == true)
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

void BHV_Fig8::setInfoVars()
{
  addInfoVars("NAV_X");
  addInfoVars("NAV_Y");
  addInfoVars("NAV_HEADING");
  addInfoVars("NAV_SPEED");
  addInfoVars("NODE_REPORT_LOCAL");
  addInfoVars("NODE_MESSAGE_LOCAL");
  addInfoVars("NODE_REPORT");
  addInfoVars("NODE_MESSAGE");
  addInfoVars("NODE_MESSAGE");
  addInfoVars(p_updates_var);
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Fig8::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  // double double_val = atof(val.c_str());
  bool handled = false;

  if ((param == "center_x") && isNumber(val))
  {
    p_lem_center_x = atof(val.c_str());
    handled = true;
  }
  else if ((param == "center_y") && isNumber(val))
  {
    p_lem_center_y = atof(val.c_str());
    handled = true;
  }
  else if ((param == "width") && isNumber(val))
  {
    p_lem_width = atof(val.c_str());
    handled = true;
  }

  else if ((param == "height") && isNumber(val))
  {
    p_lem_height = atof(val.c_str());
    handled = true;
  }
  else if ((param == "alpha") && isNumber(val))
  {
    p_lem_alpha_deg = atof(val.c_str());
    p_lem_alpha_rad = atof(val.c_str()) * M_PI / 180;
    handled = true;
  }
  else if ((param == "direction") && isNumber(val))
  {
    double dval = atof(val.c_str());
    // p_lem_direction = atof(val.c_str());
    if (dval > 0)
    {
      p_lem_direction = 1;
    }
    else
    {
      p_lem_direction = -1;
    }
    handled = true;
  }
  else if ((param == "n_points") && isNumber(val))
  {
    p_lem_n_points = atof(val.c_str());
    handled = true;
  }
  else if ((param == "capture_radius") && isNumber(val))
  {
    p_capture_radius = atof(val.c_str());
    handled = true;
  }
  else if ((param == "slip_radius") && isNumber(val))
  {
    p_slip_radius = atof(val.c_str());
    handled = true;
  }
  else if ((param == "desired_speed") && isNumber(val))
  {
    p_desired_speed = atof(val.c_str());
    handled = true;
  }
  else if ((param == "updates_var"))
  {
    p_updates_var = val;
    handled = true;
  }
  else if ((param == "cycles") && isNumber(val))
  {
    p_max_cycles = atoi(val.c_str());
    handled = true;
  }
  else if ((param == "duration") && isNumber(val))
  {
    p_max_duration = atof(val.c_str());
    handled = true;
  }
  else if (param == "debug")
  {
    if (val == "true")
    {
      p_debug = true;
    }
    handled = true;
  }

  // If not handled above, then just return false;
  return (handled);
}

void BHV_Fig8::setDebug()
{
  p_debug = true;
  time_t rawtime;
  struct tm *timeinfo;
  char time_str[80];
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", timeinfo);
  m_debug_fname = "debug_" + m_us_name + "_" + std::string(time_str) + ".txt";
}

void BHV_Fig8::initStateVars()
{
  m_osx = 0;
  m_osy = 0;
  m_osh = 0;
  m_target_x = 0;
  m_target_y = 0;
  p_lem_center_x = 0;
  p_lem_center_y = 0;
  p_lem_width = 10;
  p_lem_height = 5;
  p_lem_alpha_deg = 0;
  p_lem_alpha_rad = 0;
  p_lem_direction = 1;
  p_lem_n_points = 100;
  m_lem_cidx = 0;
  m_lem_percent = 0.0;
  p_capture_radius = 4;
  p_slip_radius = 10;
  p_desired_speed = 1;
  m_desired_heading = 0;
  p_updates_var = "LEMNISCATE_UPDATES";

  p_max_cycles = -1;   // -1 means run forever
  p_max_duration = -1; // -1 means run forever
  m_completed_cycles = 0;
  m_start_time = 0;
  m_duration_exceeded = false;
  m_cycles_exceeded = false;

  m_os_node_record = NodeRecord();

  m_first = true;
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

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_Fig8::onSetParamComplete()
{
  if (p_debug)
    setDebug();
  // Add any variables this behavior needs to subscribe for
  setInfoVars();
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_Fig8::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_Fig8::onIdleState()
{
  updateMail();
}

void BHV_Fig8::updateMail()
{
  // Here, we capture all the data which is continuously streamed, as well as new data which helps transition states

  // We are receiving the instantaneous errors from each agent, as well as listening for a new point to include in our point queue, sent from the agent whom we are following

  // Update other agent states
  if (getBufferVarUpdated("NODE_REPORT_LOCAL"))
    updateOwnshipNodeRecord();

  m_osx = getBufferDoubleVal("NAV_X");
  m_osy = getBufferDoubleVal("NAV_Y");
  m_osh = getBufferDoubleVal("NAV_HEADING");
  m_osv = getBufferDoubleVal("NAV_SPEED");

  if (getBufferVarUpdated(p_updates_var))
    updateParameters();
}

void BHV_Fig8::updateOwnshipNodeRecord()
{
  m_nr_t = getBufferTimeVal("NODE_REPORT_LOCAL");
  m_os_node_record = string2NodeRecord(getBufferStringVal("NODE_REPORT_LOCAL"));
  m_osx = m_os_node_record.getX();
  m_osy = m_os_node_record.getY();
  m_osh = m_os_node_record.getHeading();
  m_osv = m_os_node_record.getSpeed();
  m_os_color = m_os_node_record.getColor();
  m_us_name = m_os_node_record.getName();
}

void BHV_Fig8::applyParamModifier(const string &update)
{
  vector<string> key_val = parseString(update, '=');

  if (key_val.size() != 2)
    return;

  double modifier = 0;

  string key = key_val[0];

  if (strContains(update, "+="))
  {
    modifier = 1;
  }
  else
  {
    modifier = -1;
  }

  key = key.substr(0, key.size() - 1); // Remove '*=' from the key
  // dbg print everything relevant for what we are doing message, modifier, variable, and val

  if (key == "center_x")
  {
    p_lem_center_x += modifier * stod(key_val[1]);
  }
  else if (key == "center_y")
  {
    p_lem_center_y += modifier * stod(key_val[1]);
  }
  else if (key == "width")
  {
    p_lem_width += modifier * stod(key_val[1]);
  }
  else if (key == "height")
  {
    p_lem_height += modifier * stod(key_val[1]);
  }
  else if (key == "alpha")
  {
    p_lem_alpha_deg += modifier * stod(key_val[1]);
    p_lem_alpha_rad = p_lem_alpha_deg * M_PI / 180;
  }
  else if (key == "direction")
  {
    p_lem_direction = modifier;
    dbg_print("Direction: %f\n", p_lem_direction);
    m_lem_cidx = p_lem_n_points - m_lem_cidx;
  }
  else if (key == "n_points")
  {
    m_lem_percent = static_cast<double>(m_lem_cidx) / static_cast<double>(p_lem_n_points);
    int change = modifier * stod(key_val[1]);
    // error check to make sure it isn't negative
    if ((p_lem_n_points + change) <= 0)
      return;
    p_lem_n_points += change;
    // we now want to scale m_lem_cidx to be the same equivalent index in the curve
    m_lem_cidx = (m_lem_percent * p_lem_n_points);
  }
  string val = key_val[1];
}

void BHV_Fig8::updateParameters()
{
  string updates = getBufferStringVal(p_updates_var);
  vector<string> update_pairs = parseString(updates, ',');
  for (const string &update : update_pairs)
  {
    /*
      Here, we either receive a new assigned parameter update, or a modifier
      i.e. a
      center_x=10, center_y=10, width=10, height=10, alpha=10, direction=1, n_points=100
      or
      center_x+=10, center_y+=10, width-=10, height-=10, alpha+=10, direction+=1, n_points+=100
    */
    /*
      Check to see if this is a modifier or a new assignment
    */
    if (strContains(update, "+=") || strContains(update, "-="))
    {
      /*
        This only applies to a handful of parameters, so we just evaluate this logic here and in place
      */
      applyParamModifier(update);
    }
    else if (strContains(update, "="))
    {
      // We will update the parameter
      vector<string> key_val = parseString(update, '=');
      if (key_val.size() != 2)
      {
        continue;
      }
      string key = key_val[0];
      string val = key_val[1];
      setParam(key, val);
      dbg_print("Tried updating %s to %s\n", key.c_str(), val.c_str());
      // We will also now get all the points in the lemniscate, and publish them as a XYSegList
    }
  }
  m_first = true;
  dbg_print("Published lemniscate points\n");
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Fig8::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_Fig8::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_Fig8::onIdleToRunState()
{
  updateMail();
  if (m_us_name != "")
  {
    m_latest_lemniscate_seglist.clear();
    for (int i = 0; i < p_lem_n_points; i++)
    {
      double x, y;
      m_lemniscate_calculator.getPoint(x, y,
                                       p_lem_center_x, p_lem_center_y,
                                       p_lem_width, p_lem_height,
                                       p_lem_alpha_deg,
                                       p_lem_direction,
                                       static_cast<double>(i) / static_cast<double>(p_lem_n_points));
      m_latest_lemniscate_seglist.add_vertex(x, y);
    }
    m_latest_lemniscate_seglist.set_color("edge", "orange");
    m_latest_lemniscate_seglist.set_label(m_us_name + "_lemniscate");
    dbg_print("Publishing lemniscate points\n");
    postMessage("VIEW_SEGLIST", m_latest_lemniscate_seglist.get_spec(2));
    dbg_print("Published seglist: %s\n", m_latest_lemniscate_seglist.get_spec().c_str());
    m_first = true;
  }
  m_completed_cycles = 0;
  m_start_time = getBufferCurrTime();
  m_duration_exceeded = false;
  m_cycles_exceeded = false;
}

void BHV_Fig8::postErasables() {
  if (m_us_name != "")
  {
    m_latest_lemniscate_seglist.set_label_color("invisible");
    postMessage("VIEW_SEGLIST", m_latest_lemniscate_seglist.get_spec());
    XYPoint target_point(m_target_x, m_target_y);
    std::string target_label = m_us_name + "_target";
    target_point.set_label(target_label);
    target_point.set_vertex_size(5);
    target_point.set_color("fill", "invisible");
    target_point.set_color("edge", "invisible");
    postMessage("VIEW_POINT", target_point.get_spec());
    m_first = true;
  }
}

bool slipped(double dist, double slip_radius, double rel_ang)
{
  // We slip a point if the distance is within the slip radius, and is behind the agent
  // TODO: this is a bug at the moment
  return (dist < slip_radius && abs(rel_ang) > 90);
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_Fig8::onRunToIdleState()
{
  updateMail();
  postErasables();
}

IvPFunction *BHV_Fig8::getSimpleSpeedPeak(double desired_speed)
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
IvPFunction *BHV_Fig8::getSimpleHeadingPeak(double desired_heading)
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

IvPFunction *BHV_Fig8::getSimpleCoupledPeak(double desired_speed, double desired_heading)
{
  IvPFunction *crs_ipf = getSimpleHeadingPeak(desired_heading);
  IvPFunction *spd_ipf = getSimpleSpeedPeak(desired_speed);

  // Couple the heading and speed functions
  OF_Coupler coupler;
  IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);
  return ivp_function;
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction *BHV_Fig8::onRunState()
{
  updateMail();
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;

  if (m_us_name == "")
    return ipf;

  // Check termination conditions
  if (p_max_duration > 0)
  {
    if ((getBufferCurrTime() - m_start_time) > p_max_duration)
    {
      m_duration_exceeded = true;
      postMessage("FIG8_STATUS", "duration_exceeded");
      postErasables();
      setComplete();
      return 0;
    }
  }

  if (p_max_cycles > 0 && m_completed_cycles >= p_max_cycles)
  {
    m_cycles_exceeded = true;
    postMessage("FIG8_STATUS", "cycles_exceeded");
    postErasables();
    setComplete();
    return 0;
  }

  // Get the closest point on the curve
  if (m_first && m_us_name != "")
  {
    m_latest_lemniscate_seglist.clear();
    for (int i = 0; i < p_lem_n_points; i++)
    {
      double x, y;
      m_lemniscate_calculator.getPoint(x, y,
                                       p_lem_center_x, p_lem_center_y,
                                       p_lem_width, p_lem_height,
                                       p_lem_alpha_deg,
                                       p_lem_direction,
                                       static_cast<double>(i) / static_cast<double>(p_lem_n_points));
      m_latest_lemniscate_seglist.add_vertex(x, y);
    }
    m_latest_lemniscate_seglist.set_color("edge", "orange");
    m_latest_lemniscate_seglist.set_edge_size(2);
    m_latest_lemniscate_seglist.set_label(m_us_name + "_lemniscate");

    postMessage("VIEW_SEGLIST", m_latest_lemniscate_seglist.get_spec(2));

    double min_cost = 1000000000;

    for (uint64_t i = 0; i < p_lem_n_points; i++)
    {
      double x, y;
      m_lemniscate_calculator.getPoint(x, y,
                                       p_lem_center_x, p_lem_center_y,
                                       p_lem_width, p_lem_height,
                                       p_lem_alpha_deg,
                                       p_lem_direction,
                                       static_cast<double>(i) / static_cast<double>(p_lem_n_points));
      double angle_to = relAng(m_osx, m_osy, x, y);
      double rel_ang = angle_to - m_osh;
      // print all angles and headings

      double dist = hypot(m_osx - x, m_osy - y);
      // double heading_score = (abs(rel_ang) / 180);
      // double heading_score = 1-((1+cos(rel_ang*M_PI/180.0))/2.0)*0.5;
      // double cost = dist*heading_score;
      double cost = dist;
      if (cost < min_cost)
      {
        min_cost = cost;
        m_lem_cidx = i;
      }
    }

    m_first = false;
  }

  double dist_to_point = 0;
  double rel_ang = 0;
  do
  {
    // Observe the next point in the queue
    m_lem_percent = static_cast<double>(m_lem_cidx) / static_cast<double>(p_lem_n_points);
    m_lemniscate_calculator.getPoint(m_target_x, m_target_y,
                                     p_lem_center_x, p_lem_center_y,
                                     p_lem_width, p_lem_height,
                                     p_lem_alpha_deg,
                                     p_lem_direction,
                                     m_lem_percent);
    // If we capture this point, or slip this point, we remove it from the queue
    dist_to_point = hypot(m_osx - m_target_x, m_osy - m_target_y);
    rel_ang = rel_ang = relAng(m_osx, m_osy, m_target_x, m_target_y) - m_osh;

    if (dist_to_point < p_capture_radius || slipped(dist_to_point, p_slip_radius, rel_ang))
    {
      m_lem_cidx++;
      if (m_lem_cidx >= p_lem_n_points)
      {
        m_lem_cidx = 0;
        m_completed_cycles++;
        string status = "cycles_completed=" + intToString(m_completed_cycles);
        if (p_max_cycles > 0)
        {
          status += ",cycles_remaining=" + intToString(p_max_cycles - m_completed_cycles);
        }
        postMessage("FIG8_STATUS", status);
      }
    }
    else
    {
      // If we can track the next point in the queue without slipping or capturing it, we exit this loop, i.e. this point hasn't been captured, and has not been slipped
      break;
    }
    // Otherwise, we continue to inspect a qualifying point
  } while (true);

  m_desired_heading = relAng(m_osx, m_osy, m_target_x, m_target_y);

  XYPoint target_point(m_target_x, m_target_y);
  std::string target_label = m_us_name + "_target";
  target_point.set_label(target_label);
  target_point.set_vertex_size(5);
  target_point.set_color("fill", "orange");
  target_point.set_color("edge", "black");
  postMessage("VIEW_POINT", target_point.get_spec());

  ipf = getSimpleCoupledPeak(p_desired_speed, m_desired_heading);

  if (ipf)
    ipf->setPWT(m_priority_wt);

  return (ipf);
}
