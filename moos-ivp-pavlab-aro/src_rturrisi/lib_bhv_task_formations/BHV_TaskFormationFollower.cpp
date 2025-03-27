
/* **************************************************************
  NAME: Raymond Turrisi
  ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
  FILE: BHV_TaskFormationFollower.cpp
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
#include <iostream>
#include <cmath>
#include "BHV_TaskFormationFollower.h"
#include "MBUtils.h"
#include "MacroUtils.h"
#include "AngleUtils.h"
#include "dubin.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include "XYPolygon.h"
#include "XYFormatUtilsSegl.h"
#include <fstream>
#include <iostream>

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_TaskFormationFollower::BHV_TaskFormationFollower(IvPDomain domain) : IvPTaskBehavior(domain)
{
  m_turn_radius = 3; // meters
}

//-----------------------------------------------------------
// Procedure: onHelmStart()

void BHV_TaskFormationFollower::onHelmStart()
{
  if (m_update_var == "")
    return;

  string alert_request = "type=" + m_task_type;
  alert_request += ", var=" + m_update_var;
  postMessage("TM_ALERT_REQUEST", alert_request);
}

//-----------------------------------------------------------
// Procedure: setParam

bool BHV_TaskFormationFollower::setParam(string param, string value)
{
  if (IvPTaskBehavior::setParam(param, value))
    return (true);
  param = tolower(param);
  if ((param == "turn_radius") && isNumber(value))
  {
    m_turn_radius = atof(value.c_str());
    m_turn_radius_set = true;
    return (true);
  }
  else if ((param == "pos_idx") && isNumber(value))
  {
    m_pos_idx = atof(value.c_str());
    return (true);
  }

  return (false);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_TaskFormationFollower::onIdleState()
{
  IvPTaskBehavior::onGeneralIdleState();
  return;
}

//-----------------------------------------------------------
// Procedure: onRunState()
//    States: spawned : noroster : roster : bidding

IvPFunction *BHV_TaskFormationFollower::onRunState()
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

double BHV_TaskFormationFollower::getTaskBid()
{
  // TODO: Get the projected point from the formation geometry class and utilities - the only contact should be the leader

  double heading_to_target = relAng(m_osx, m_osy, m_cnx, m_cny);
  double precision = 1;

  double goal_heading_rad = (90 - heading_to_target) * M_PI / 180;
  double os_heading_rad = (90 - m_osh) * M_PI / 180;
  // Angle difference between heading and heading to target
  DubinsPath dubins_calculator;

  string seglist_str = dubins_calculator.findOptimalWaypoints(Point(m_osx, m_osy), os_heading_rad,
                                                              Point(m_cnx, m_cny), goal_heading_rad,
                                                              m_turn_radius, m_turn_radius, m_turn_radius, precision);
  XYSegList predicted_trajectory = string2SegList(seglist_str);

  predicted_trajectory.add_vertex(m_cnx, m_cny);
  predicted_trajectory.set_color("edge", "yellow");
  predicted_trajectory.set_duration(10);
  predicted_trajectory.set_label(m_us_name + "_dubins_formation");
  postMessage("VIEW_SEGLIST", predicted_trajectory.get_spec(2));
  double dist = predicted_trajectory.length();
  return (dist);
}

//-----------------------------------------------------------
// Procedure: applyFlagMacros()

vector<VarDataPair> BHV_TaskFormationFollower::applyFlagMacros(vector<VarDataPair> flags)
{
  // ofstream dbg_file;
  // dbg_file.open(m_us_name + "_helmtask_dbg.txt", ios_base::app);
  for (unsigned int i = 0; i < flags.size(); i++)
  {
    string var = flags[i].get_var();
    if (flags[i].is_string())
    {
      string sdata = flags[i].get_sdata();
      sdata = macroExpand(sdata, "CONTACT", tolower(m_contact));
      int next_pos_idx = m_pos_idx + 1;
      sdata = macroExpand(sdata, "POS_IDX", to_string(next_pos_idx));
      postMessage("FORMATION_IDX", to_string(m_pos_idx));
      flags[i].set_sdata(sdata, true);
    }
    // dbg_file << "Var: " << var << ", SData: " << flags[i].get_sdata() << endl;
  }
  // dbg_file.close();

  return (flags);
}
