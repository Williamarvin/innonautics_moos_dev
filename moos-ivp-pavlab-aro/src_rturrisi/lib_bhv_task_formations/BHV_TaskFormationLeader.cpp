
/* **************************************************************
  NAME: Raymond Turrisi
  ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
  FILE: BHV_TaskFormationLeader.cpp
  CIRC: November 2023
  DESC:
    Constructs a bid for the convoy ordering problem with a dubins
    turn. The total odometry of this proposed dubins turn / path
    is the bid.

  LICENSE:
    This is unreleased BETA code. No permission is granted or
    implied to use, copy, modify, and distribute this software
    except by the author(s), or those designated by the author.
************************************************************** */

#include <cstdlib>
#include <cmath>
#include "BHV_TaskFormationLeader.h"
#include "MBUtils.h"
#include "MacroUtils.h"
#include "AngleUtils.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include "XYPolygon.h"
#include "XYFormatUtilsSegl.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_TaskFormationLeader::BHV_TaskFormationLeader(IvPDomain domain) : IvPTaskBehavior(domain)
{
  // Initialize state variables
  m_ptx = 0;
  m_pty = 0;
  m_turn_radius = 3;

  m_ptx_set = false;
  m_pty_set = false;
  m_turn_radius_set = false;

  m_consider_contacts = false;
}

//-----------------------------------------------------------
// Procedure: onHelmStart()

void BHV_TaskFormationLeader::onHelmStart()
{
  // if(m_update_var == "")
  //   return;

  string alert_request = "type=" + m_task_type;
  alert_request += ", var=" + m_update_var;
  postMessage("TM_ALERT_REQUEST", alert_request);
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_TaskFormationLeader::setParam(string param, string value)
{
  if (IvPTaskBehavior::setParam(param, value))
    return (true);

  param = tolower(param);
  if ((param == "waypt_x") && isNumber(value))
  {
    m_ptx = atof(value.c_str());
    m_ptx_set = true;
    return (true);
  }
  else if ((param == "waypt_y") && isNumber(value))
  {
    m_pty = atof(value.c_str());
    m_pty_set = true;
    return (true);
  }
  else if ((param == "turn_radius") && isNumber(value))
  {
    m_turn_radius = atof(value.c_str());
    m_turn_radius_set = true;
    return (true);
  }
  else if (param == "waypt")
  {
    string xstr = biteStringX(value, ',');
    string ystr = value;
    if (isNumber(xstr) && isNumber(ystr))
    {
      m_ptx = atof(xstr.c_str());
      m_ptx_set = true;
      m_pty = atof(ystr.c_str());
      m_pty_set = true;
      return (true);
    }
  }
  else if (param == "consider_contacts")
    return (setBooleanOnString(m_consider_contacts, value));

  return (false);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_TaskFormationLeader::onIdleState()
{
  IvPTaskBehavior::onGeneralIdleState();
  return;
}

//-----------------------------------------------------------
// Procedure: onRunState()
//    States: alerted : noroster : roster : bidding :
//            bidwon : bidlost : abstain

IvPFunction *BHV_TaskFormationLeader::onRunState()
{
  IvPTaskBehavior::onGeneralRunState();
  return (0);
}

//-----------------------------------------------------------
// Procedure: getTaskBid()

double pmod(double a, double b)
{
  double r = fmod(a, b);
  if (r < 0)
  {
    r += b;
  }
  return r;
};

double BHV_TaskFormationLeader::getTaskBid()
{
  // Get the differences between the state and the desired point
  double dx = m_ptx - m_osx; // meters
  double dy = m_pty - m_osy; // meters

  double heading_to_target = relAng(m_osx, m_osy, m_ptx, m_pty);
  double precision = 1;

  DubinsPath dubins_calculator;

  double goal_heading_rad = (90 - heading_to_target) * M_PI / 180;
  double os_heading_rad = (90 - m_osh) * M_PI / 180;

  string seglist_str = dubins_calculator.findOptimalWaypoints(Point(m_osx, m_osy), os_heading_rad,
                                                              Point(m_ptx, m_pty), goal_heading_rad,
                                                              m_turn_radius, m_turn_radius, m_turn_radius, precision);
  XYSegList predicted_trajectory = string2SegList(seglist_str);

  // Once we have our center of rotation placed, we zip around the turning radius, discretely, determining whether or not we have a trajectory which does not intersect with the circle
  predicted_trajectory.add_vertex(m_ptx, m_pty);
  predicted_trajectory.set_color("edge", "red");
  predicted_trajectory.set_time(10);
  predicted_trajectory.set_duration(10);
  predicted_trajectory.set_label(m_us_name + "_dubins_wpt");
  predicted_trajectory.set_label_color("invisible");
  postMessage("VIEW_SEGLIST", predicted_trajectory.get_spec());

  double dist = predicted_trajectory.length();
  return (dist);
}

//-----------------------------------------------------------
// Procedure: applyFlagMacros()

vector<VarDataPair> BHV_TaskFormationLeader::applyFlagMacros(vector<VarDataPair> flags)
{
  string ptx_str = doubleToStringX(m_ptx, 2);
  string pty_str = doubleToStringX(m_pty, 2);

  for (unsigned int i = 0; i < flags.size(); i++)
  {
    if (flags[i].is_string())
    {
      string sdata = flags[i].get_sdata();
      sdata = macroExpand(sdata, "PTX", ptx_str);
      sdata = macroExpand(sdata, "PTY", pty_str);
      flags[i].set_sdata(sdata, true);
    }
  }

  return (flags);
}
