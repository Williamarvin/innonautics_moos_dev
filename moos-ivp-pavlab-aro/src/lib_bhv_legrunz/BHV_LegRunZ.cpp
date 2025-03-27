/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng MIT                           */
/*    FILE: BHV_LegRunZ.cpp                                      */
/*    DATE: May 30th, 2023                                       */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/

#include <cmath> 
#include <cstdlib>
#include <iostream>
#include "BHV_LegRunZ.h"
#include "MBUtils.h"
#include "MacroUtils.h"
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "BuildUtils.h"
#include "ZAIC_PEAK.h"
#include "ZAIC_SPD.h"
#include "OF_Coupler.h"
#include "VarDataPairUtils.h"
#include "ColorParse.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor()

BHV_LegRunZ::BHV_LegRunZ(IvPDomain gdomain) : IvPBehavior(gdomain)
{
  // =================================================
  // Superclass vars
  m_descriptor = "bhv_legrun";  
  m_domain     = subDomain(m_domain, "course,speed");

  // =================================================
  // Config vars
  m_cruise_spd  = 0;  // meters/second
  m_patience    = 50;
  
  m_mid_pct = 25;
  m_offboard_tgap = 0;
  m_init_leg_mode = "fixed";
  
  // Visual Hint Defaults for the leg/turn paths
  m_hints.setMeasure("vertex_size", 3);
  m_hints.setMeasure("edge_size", 1);
  m_hints.setColor("vertex_color", "dodger_blue");
  m_hints.setColor("edge_color", "gray60");
  m_hints.setColor("label_color", "off");

  // Visual Hint Defaults for the next/track points
  m_hints.setMeasure("nextpt_vertex_size", 3);
  m_hints.setColor("nextpt_vertex_color", "yellow");
  m_hints.setColor("nextpt_label_color", "aqua");

  // Visual Hint Defaults for leg end points
  m_hints.setMeasure("legpt_vertex_size", 7);
  m_hints.setColor("legpt_vertex_color", "orange");
  m_hints.setColor("legpt_label_color", "off");

  // Visual Hint Defaults for the turn previews
  m_hints.setMeasure("turn_vertex_size", 2);
  m_hints.setMeasure("turn_edge_size", 1);
  m_hints.setColor("turn_vertex_color", "gray70");
  m_hints.setColor("turn_edge_color", "gray30");
  m_hints.setColor("turn_label_color", "off");

  // Visual Hint Defaults for the ring
  m_hints.setMeasure("ring_vertex_size", 0);
  m_hints.setMeasure("ring_edge_size", 1);
  m_hints.setColor("ring_vertex_color", "off");
  m_hints.setColor("ring_edge_color", "gray50");
  m_hints.setColor("ring_label_color", "off");

  m_leg_spds_repeat = false;
  m_leg_spds_onturn = false;

  m_coord_extrap = true;
  m_coord_onleg  = true;
  
  // =================================================
  // State Vars
  m_mid_event_yet   = false;
  m_preview_pending = true;
  
  m_leg_count = 0;
  m_leg_count1 = 0;
  m_leg_count2 = 0;
  
  m_leg_spds_ix = 0;
  m_leg_spds_curr = -1; // Neg means use cruise spd
  
  m_mode = "init";

  // The completed and perpetual vars are initialized in superclass
  // but we initialize here just to be safe and clear.
  m_completed = false; 
  m_perpetual = true;

  addInfoVars("NAV_X, NAV_Y, NAV_SPEED");
  addInfoVars("LR_REMX_DIST, LR_DIMS");

  // turn_spd is used if multi-vehicle coordinatin is used
  m_turn_coord_spd = -1;

  m_max_spd = m_domain.getVarHigh("speed");

  m_spd_modulator.setFullStopRng(-10);
  m_spd_modulator.setSlowerRng(-0.1);
  m_spd_modulator.setFasterRng(0.1);
  m_spd_modulator.setFullLagRng(10);
  m_spd_modulator.setLagSpeedDelta(0.6);
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_LegRunZ::setParam(string param, string value) 
{
  if(param == "speed")
    return(setDoubleRngOnString(m_cruise_spd, value, 0, m_max_spd));
  else if(param == "leg_spds")
    return(handleConfigLegSpeed(value));
  else if(param == "leg_spds_repeat")
    return(setBooleanOnString(m_leg_spds_repeat, value));
  else if(param == "leg_spds_onturn")
    return(setBooleanOnString(m_leg_spds_onturn, value));
  else if(param == "cycleflag") 
    return(addVarDataPairOnString(m_cycle_flags, value));
  else if(param == "wptflag") 
    return(addVarDataPairOnString(m_wpt_flags, value));
  else if(param == "legflag") 
    return(addVarDataPairOnString(m_leg_flags, value));
  else if(param == "midflag") 
    return(addVarDataPairOnString(m_mid_flags, value));
  else if(param == "start_leg_flag") 
    return(addVarDataPairOnString(m_start_leg_flags, value));
  else if(param == "start_turn_flag") 
    return(addVarDataPairOnString(m_start_turn_flags, value));
  
  else if(param == "lead") {
    m_wpteng_turn.setParam(param, value);
    return(m_wpteng_legs.setParam(param, value));
  }
  else if(param == "lead_damper") {
    m_wpteng_turn.setParam(param, value);
    return(m_wpteng_legs.setParam(param, value));
  }
  else if(param == "capture_line") {
    m_wpteng_turn.setParam(param, value);
    return(m_wpteng_legs.setParam(param, value));
  }
  else if(param == "repeat")
    return(m_wpteng_legs.setParam(param, value));
  else if((param == "radius") || (param == "capture_radius")) {
    m_wpteng_turn.setParam(param, value);
    return(m_wpteng_legs.setParam(param, value));
  }
  else if(param == "slip_radius") {
    m_wpteng_turn.setParam(param, value);
    return(m_wpteng_legs.setParam(param, value));
  }
  else if(param == "mid_pct") 
    return(setDoubleStrictRngOnString(m_mid_pct, value, 0, 100));
  else if(param == "offboard_tgap") 
    return(setNonNegDoubleOnString(m_offboard_tgap, value));
  
  else if(param == "patience")
    return(setDoubleClipRngOnString(m_patience, value, 1, 99));
  else if(param == "coord")
    return(handleConfigCoord(value));
  else if(param == "visual_hints") 
    return(m_hints.setHints(value));

  else if(param == "coord_extrap")
    return(setBooleanOnString(m_coord_extrap, value));
  else if(param == "coord_onleg")
    return(setBooleanOnString(m_coord_onleg, value));
  else if(param == "init_leg_mode") {
    if ((value == "fixed") || (value == "close_turn") ||
	(value == "far_turn")) {
      m_init_leg_mode = value;
      return(true);
    } else
      return(false);
  }

  // All other params are handled in the LegRun level
  m_preview_pending = true;
  return(m_legrun.setParam(param, value));
}

//-----------------------------------------------------------
// Procedure: onSetParamComplete()

void BHV_LegRunZ::onSetParamComplete()
{
  if(m_wpteng_legs.getMaxRepeats() > 0)
    IvPBehavior::setParam("perpetual", "true");

  if(!m_legrun.valid())
    postWMessage("Invalid or unset legrun");
  else
    postRetractWMessage("Invalid or unset legrun");

  // Handle change in core leg run. 
  if(m_legrun.legModified()) {
    XYSegList segl;
    segl.add_vertex(m_legrun.getPoint1());
    segl.add_vertex(m_legrun.getPoint2());
    m_wpteng_legs.setSegList(segl, true);

    if((m_mode == "turn1") || (m_mode == "turn2")) {
      postTurnSegList(false);
      postSteerPoints(false);
    }
    postAllSegList();
    postLegRing();
    setMode("init");
    m_legrun.setLegModified(false);
    postLengthSpecs(true);

    double td1 = m_legrun.getTurn1Len();
    double td2 = m_legrun.getTurn2Len();
    double leg = m_legrun.getLegLen();    
    m_ring_master.setDimensionsLR("ownship", leg, td1, td2);
  }

  regulateOffboardMessage("LR_DIMS", m_offboard_tgap);
  regulateOffboardMessage("LR_REMX_DIST", m_offboard_tgap);
  
  postConfigStatus();
}

//-----------------------------------------------------------
// Procedure: onRunToIdleState()
//      Note: Invoked automatically by the helm when the behavior
//            first transitions from the Running to Idle state.

void BHV_LegRunZ::onRunToIdleState() 
{
  eraseAllViewables();

  setMode("idle");
  m_wpteng_legs.resetCPA();
  m_wpteng_turn.resetCPA();
}

//-----------------------------------------------------------
// Procedure: onIdleToRunState()
//      Note: Invoked automatically by the helm when the behavior
//            first transitions from the Idle to Running state.

void BHV_LegRunZ::onIdleToRunState() 
{
  m_preview_pending = true;
  m_mode = "init";
  initLegMode();
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_LegRunZ::onRunState() 
{
  // Set m_osx, m_osy, m_osv, osh
  if(!updateInfoIn()) {
    eraseAllViewables();
    return(0);
  }

  bool okleg = checkValidLegRun();
  if(!okleg)
    return(0);
  
  if(m_mode == "init")
    initLegMode();

  checkProvisional();
  
  if(m_preview_pending)
    postTurnPreview();
  
  // Pass 1: Generate a track point. A return of false means leg/turn
  // was emptry (err) or completed. The latter means a mode switch is
  // coming.
  bool ok = false;
  if((m_mode == "leg1") || (m_mode == "leg2"))
    ok = onRunStateLegMode();
  else if((m_mode == "turn1") || (m_mode == "turn2"))
    ok = onRunStateTurnMode();

  handleModeSwitch();

  // Pass 2: If pass 1 was a mode switch, try 2nd pass.
  if(!ok) {
    if((m_mode == "leg1") || (m_mode == "leg2"))
      ok = onRunStateLegMode();
    else if((m_mode == "turn1") || (m_mode == "turn2"))
      ok = onRunStateTurnMode();
  }

  if(!ok)
    return(0);

  if(m_coord != "") {
    postPositionStatus();
    postLengthSpecs();

    //setGrpDistFromBeg();
    m_ring_master.updateCenter();
    double gpa = m_ring_master.getCenterDegs();
    setTurnCoordSpd(gpa);
    postGrpAvgPoint(gpa);
  }
  
  IvPFunction *ipf = buildOF();
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}

//-----------------------------------------------------------
// Procedure: onRunStateTurnMode()

bool BHV_LegRunZ::onRunStateTurnMode() 
{
  m_wpteng_turn.setNextWaypoint(m_osx, m_osy);    

  m_trackpt = m_wpteng_turn.getTrackPoint();
  m_nextpt  = m_wpteng_turn.getNextPoint();

  if(m_wpteng_turn.hasCompleted()) {
    advanceModePending();
    postTurnSegList(false);
    postSteerPoints(false);
    m_wrap_detector.reset();
    m_odometer.reset();
    m_odometer.unpause();
    return(false);
  }

  if(m_wpteng_turn.hasAdvanced())
    postTurnSegList(true);

  postSteerPoints(true);
  postLegPoints(true);
    
  return(true); 
}

//-----------------------------------------------------------
// Procedure: onRunStateLegMode()

bool BHV_LegRunZ::onRunStateLegMode() 
{
  // Sanity check
  if(m_wpteng_legs.size() == 0)
    return(false);

  // Part 1: Apply ownship position to the engine
  m_wpteng_legs.setNextWaypoint(m_osx, m_osy);

  // Part 2: Handle advanced, cycled and completed
  if(m_wpteng_legs.hasAdvanced()) {
    advanceModePending();
    m_odometer.pause();
  }

  if(m_wpteng_legs.hasCycled())
    postFlags(m_cycle_flags);
    
  if(m_wpteng_legs.hasCompleted()) {
    setComplete();
    if(m_perpetual)
      m_wpteng_legs.resetForNewTraversal();
    postLegSegList(false);
    postSteerPoints(false);
    return(false);
  }

  // Advancement may mean mode switch so don't pub legflags
  if(!m_wpteng_legs.hasAdvanced())
    postFlags(m_leg_flags);
  
  // Part 3: Not completed so update core vars and visuals
  m_trackpt = m_wpteng_legs.getTrackPoint();
  m_nextpt  = m_wpteng_legs.getNextPoint();
  
  postLegSegList(true);
  postSteerPoints(true);
  postLegPoints(true);

  // Part 4: Check for midflag conditions and maybe post
  if(m_leg_count > 0) {
    double pct_leg = 100 * m_wpteng_legs.pctToNextWpt(m_osx, m_osy);
    if((pct_leg > m_mid_pct) && !m_mid_event_yet) {
      postFlags(m_mid_flags, true);
      m_mid_event_yet = true;
    }
  }
  
  return(true);
}

//-----------------------------------------------------------
// Procedure: setMode()

void BHV_LegRunZ::setMode(string mode)
{
  if((mode != "leg1") && (mode != "turn1") &&
     (mode != "leg2") && (mode != "turn2") &&
     (mode != "init") && (mode != "idle"))
    return;
 
  m_mode = mode;
  postMessage("LR_MODE", m_mode);

  double utc = getBufferCurrTime();
  string report = "vname=" + m_us_name + ",mode=" + m_mode;
  report += ",utc=" + doubleToStringX(utc);
  
  postMessage("LR_MODE_REPORT", report);
}

//-----------------------------------------------------------
// Procedure: checkProvisional()
//            When the mode is leg1 or leg2, and provisional,
//            a switch is considered to the opposite leg mode
//            if two conditions are met:
//            (1) the vehicle is sufficiently far from the leg
//            line and heading in direction that would result in
//            a near 180 degree turn
//            (2) the vehicle is out-of-sync with the modes of
//            the other vehicles.

void BHV_LegRunZ::checkProvisional()
{
  if(!m_provisional)
    return;
}

//-----------------------------------------------------------
// Procedure: checkValidLegRun()

bool BHV_LegRunZ::checkValidLegRun()
{
  if(!m_legrun.valid())
    return(false);
  return(true);
}

//-----------------------------------------------------------
// Procedure: initLegMode()

void BHV_LegRunZ::initLegMode()
{
  // Sanity check
  if(m_mode != "init")
    return;
  
  if(m_init_leg_mode == "fixed") {
    setMode("leg1");
    m_wpteng_legs.setCurrIndex(0);
    return;
  }
  
  XYPoint p1 = m_legrun.getPoint1();
  XYPoint p2 = m_legrun.getPoint2();
  double  d1 = hypot(m_osx - p1.x(), m_osy - p1.y());
  double  d2 = hypot(m_osx - p2.x(), m_osy - p2.y());

  // set the index of the leg waypoint based on
  // user-set parameter
  if (m_init_leg_mode == "far_turn") {
    if(d1 > d2) { 
      setMode("leg1");
      m_wpteng_legs.setCurrIndex(0);
    }
    else {
      setMode("leg2");
      m_wpteng_legs.setCurrIndex(1);
    }
    
  } else if (m_init_leg_mode == "close_turn") {
    if(d1 < d2) { 
      setMode("leg1");
      m_wpteng_legs.setCurrIndex(0);
    }
    else {
      setMode("leg2");
      m_wpteng_legs.setCurrIndex(1);
    }
  } else {
    postEMessage("Leg dir not initialized properly, check init_leg_mode param.");
  }
  
}

//-----------------------------------------------------------
// Procedure: handleModeSwitch()

void BHV_LegRunZ::handleModeSwitch()
{
  // Sanity checks
  if((m_mode_pending == "") || (m_mode_pending == m_mode))
    return;

  setMode(m_mode_pending);

  m_mode_pending  = "";
  m_mid_event_yet = false;

  // Entering turn1 implies a leg1 vertex arrival/counter++
  if(m_mode == "turn1") {
    m_leg_count++;
    m_leg_count1++;
    initTurnPoints();  
  }
  else if(m_mode == "turn2") {
    m_leg_count++;
    m_leg_count2++;
    initTurnPoints();  
  }

  if((m_mode == "leg1") || (m_mode == "leg2")) {
    postFlags(m_start_leg_flags);
  }
  
  else if((m_mode == "turn1") || (m_mode == "turn2")) {
    postFlags(m_start_turn_flags);
  }
  
  updateLegSpd();
}

//-----------------------------------------------------------
// Procedure: updateLegSpd()
//      Legs: o Speed at by default is m_cruise_spd
//            o If no speed schedule is specified, the var
//              m_leg_spds_curr is -1, indicating the the
//              cruise speed will be used.
//            o If speed schedule specified, we pick the
//              proper speed here from the schedule.
//            o If schedule does not repeat and it has been
//              exhausted, m_leg_spds_curr reverts to -1
//     Turns: o If m_leg_spds_onturn is false (default) the
//              speed during a turn is the m_cruise_spd
//            o If m_leg_spds_onturn is true, then borrow
//              the most recent leg speed.


void BHV_LegRunZ::updateLegSpd()
{
  // Part 1: If we just entered a leg mode, perhaps set the
  // m_leg_spds_curr from a schedule, if the schedule exists
  // and has not yet been exhausted
  if((m_mode == "leg1") || (m_mode == "leg2")) {

    // A -1 value say leg spd schedule not in use
    m_leg_spds_curr = -1;

    //cout << "ee: Total leg_spds:" << m_leg_spds.size() << endl;
    //cout << "ee: m_leg_spds_ix:" << m_leg_spds_ix << endl;
    
    if(m_leg_spds.size() > 0) {
      if(m_leg_spds_ix >= (int)(m_leg_spds.size())) {
	if(m_leg_spds_repeat) {
	  m_leg_spds_ix = 0;
	  m_leg_spds_curr = m_leg_spds[0];
	}
	else
	  m_leg_spds.clear(); // schedule exhausted
      }
      else {
	m_leg_spds_curr = m_leg_spds[m_leg_spds_ix];
	//cout << "ee: m_leg_spds_curr(1)=" << m_leg_spds_curr << endl;
	m_leg_spds_ix++;
      }
    }
  }
    
  // Part 2: If we just entered one of the turn modes, keep the
  // m_leg_spds_curr set to whatever was prevailing during the
  // previous leg, unless m_leg_spds_onturn is fasle.
  if((m_mode == "turn1") || (m_mode == "turn2")) {
    if(!m_leg_spds_onturn)
      m_leg_spds_curr = -1;
  }
  //cout << "ee: m_leg_spds_curr(2)=" << m_leg_spds_curr << endl;
  
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_LegRunZ::onIdleState() 
{
  if(!updateInfoIn()) 
    return;
}

//-----------------------------------------------------------
// Procedure: updateInfoIn()
//   Purpose: Update info need by the behavior from the info_buffer.
//            Error or warning messages can be posted.
//   Returns: true if no vital info is missing from the info_buffer.
//            false otherwise.
//      Note: By posting an EMessage, this sets the state_ok member
//            variable to false which will communicate the gravity
//            of the situation to the helm.

bool BHV_LegRunZ::updateInfoIn()
{
  // ==========================================================
  // Part 1: Update Ownship position and speed from the buffer
  // ==========================================================
  bool ok1, ok2, ok3;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  m_osv = getBufferDoubleVal("NAV_SPEED", ok3);

  m_odometer.updateDistance(m_osx, m_osy);
  m_wrap_detector.updatePosition(m_osx, m_osy);
  
  // Must get ownship position from InfoBuffer
  if(!ok1 || !ok2) {
    postEMessage("No ownship X/Y info in info_buffer.");
    return(false);
  } 

  // If NAV_SPEED info is not found in the info_buffer, its
  // not a show-stopper. A warning will be posted.
  if(!ok3)
    postWMessage("No ownship speed info in info_buffer");
  if(m_coord == "")
    return(true);
  
  // ==========================================================
  // Part 2: Update other vehicles' legrun length info
  // ==========================================================
  vector<string> svector = getBufferStringVector("LR_DIMS", ok1);
  for(unsigned int i=0; i<svector.size(); i++) {
    string report = svector[i];
    string vname = tokStringParse(report, "vname");
    string std1 = tokStringParse(report, "td1");
    string std2 = tokStringParse(report, "td2");
    string sleg = tokStringParse(report, "leg");
    if((vname!="") && (std1!="") && (std2!="") && (sleg!="")) {
      double td1  = atof(std1.c_str());
      double td2 = atof(std2.c_str());
      double leg = atof(sleg.c_str());

      m_ring_master.setDimensionsLR(vname, leg, td1, td2);
    }
  }
  
  // ==========================================================
  // Part 3: Update RingMaster
  // ==========================================================
  double utc = getBufferCurrTime();
  m_ring_master.setCurrUTC(utc);

  // Part 1: TurnDist info from InfoBuffer to RingMaster
  bool ok = true;
  svector = getBufferStringVector("LR_REMX_DIST", ok);
  for(unsigned int i=0; i<svector.size(); i++) {
    string report = svector[i];
    string vname = tokStringParse(report, "vname");
    string svspd = tokStringParse(report, "osv");
    string sdegs = tokStringParse(report, "rd");
    double degs = atof(sdegs.c_str());
    double vspd = atof(svspd.c_str());

    m_ring_master.updatePosCN(vname, vspd, degs, utc);
  }
  //m_ring_master.updateCenter();

  unsigned int rm_total_stale = m_ring_master.getTotalStale();
  postMessage("LR_TOTAL_STALE", rm_total_stale);
  
  return(true);
}

//-----------------------------------------------------------
// Procedure: buildOF()

IvPFunction *BHV_LegRunZ::buildOF() 
{
  IvPFunction *spd_ipf = 0;

  XYPoint trackpt = m_wpteng_legs.getTrackPoint();
  if(m_mode == "turn")
    trackpt = m_wpteng_turn.getTrackPoint();
  
  // ========================================
  // Part 1: Build the Speed ZAIC
  // ========================================

  double now_speed = m_cruise_spd;
  if(m_leg_spds_curr > 0)
    now_speed = m_leg_spds_curr;
  if(m_turn_coord_spd >= 0)
    now_speed = m_turn_coord_spd;

  postMessage("LR_TCOORD_SPD", m_turn_coord_spd);
  postMessage("LR_NOW_SPD", now_speed);

  ZAIC_SPD spd_zaic(m_domain, "speed");
  spd_zaic.setParams(now_speed, 0.1, now_speed+0.4, 70, 20);
  spd_ipf = spd_zaic.extractIvPFunction();
  if(!spd_ipf)
    postWMessage("Failure on the SPD ZAIC via ZAIC_SPD utility");
  
  // ========================================
  // Part 2: Build the Course ZAIC
  // ========================================
  double tpx = m_trackpt.x();
  double tpy = m_trackpt.y();
  double rel_ang_to_trk_pt = relAng(m_osx, m_osy, tpx, tpy);
  
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setValueWrap(true);
  crs_zaic.setParams(rel_ang_to_trk_pt, 0, 180, 50, 0, 100);
  int ix = crs_zaic.addComponent();
  crs_zaic.setParams(m_osh, 30, 180, 5, 0, 20, ix);
  
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);
  if(!crs_ipf) 
    postWMessage("Failure on the CRS ZAIC");
  
  // ========================================
  // Part 3: Couple the Two Functions
  // ========================================
  OF_Coupler coupler;
  double crs_pct = m_patience;
  double spd_pct = 100-m_patience;
  
  IvPFunction *ipf = coupler.couple(crs_ipf, spd_ipf, crs_pct, spd_pct);
  if(!ipf)
    postWMessage("Failure on the CRS_SPD COUPLER");

  return(ipf);
}

//-----------------------------------------------------------
// Procedure: initTurnPoints()

bool BHV_LegRunZ::initTurnPoints()
{
  if((m_mode != "turn1") && (m_mode != "turn2"))
    return(false);
  
  XYSegList turn_segl;
  if(m_mode == "turn1") 
    turn_segl = m_legrun.initTurnPoints1();
  else 
    turn_segl = m_legrun.initTurnPoints2();
  
  m_wpteng_turn.resetState();
  m_wpteng_turn.setSegList(turn_segl);

  postTurnSegList();

  return(true);
}

//-----------------------------------------------------------
// Procedure: postAllSegList()

void BHV_LegRunZ::postAllSegList(bool active)
{
  return;
  XYSegList turn_segl1 = m_legrun.initTurnPoints1();
  XYSegList turn_segl2 = m_legrun.initTurnPoints2();
  turn_segl2.set_label("full_path");

  if(!active) {
    postMessage("VIEW_SEGLIST", turn_segl2.get_spec_inactive());
    return;
  }
  
  for(unsigned int i=0; i<turn_segl1.size(); i++)
    turn_segl2.add_vertex(turn_segl1.get_point(i));

  if(turn_segl2.size() == 0)
    return;
  
  string spec = turn_segl2.get_spec();
  postMessage("VIEW_SEGLIST", spec);  
}

//-----------------------------------------------------------
// Procedure: postLegSegList()

void BHV_LegRunZ::postLegSegList(bool active)
{
  return;
  XYSegList seglist = m_wpteng_legs.getSegList();
  seglist.set_label(m_us_name + "_" + m_descriptor);

  if(!active)
    postMessage("VIEW_SEGLIST", seglist.get_spec_inactive());
  else {
    applyHints(seglist, m_hints);
    postMessage("VIEW_SEGLIST", seglist.get_spec());
  }  
}

//-----------------------------------------------------------
// Procedure: postLegRing()

void BHV_LegRunZ::postLegRing(bool active)
{
  return;
  XYPolygon ringpoly;

  XYPoint cpt = m_legrun.getCenterPt();
  double cx = cpt.x();
  double cy = cpt.y();
  double diam = m_legrun.getLegLen();
  cx = (int)cx;
  cy = (int)cy;
  
  ringpoly.setRadial(cx,cy, diam/2, 36, 0.01);
  ringpoly.set_label("center_joust");
  
  if(!active)
    postMessage("VIEW_POLYGON", ringpoly.get_spec_inactive());
  else {
    applyHints(ringpoly, m_hints, "ring");
    postMessage("VIEW_POLYGON", ringpoly.get_spec());
  }  
}

//-----------------------------------------------------------
// Procedure: postTurnSegList()

void BHV_LegRunZ::postTurnSegList(bool active)
{
  XYSegList segl = m_wpteng_turn.getSegList();
  segl.set_label(m_us_name + "_turn");

  if(!active) {
    postMessage("VIEW_SEGLIST", segl.get_spec_inactive());
    return;
  }

#if 0
  XYPoint fpt = segl.get_first_point();
  XYPoint lpt = segl.get_last_point();

  fpt.set_vertex_size(38);
  fpt.set_vertex_color("dodger_blue");
  fpt.set_label_color("off");
  fpt.set_label(m_us_name + "_tfpt");
  fpt.set_duration(600);
  string fpt_spec = fpt.get_spec();
  postMessage("VIEW_POINT", fpt_spec);

  lpt.set_vertex_size(28);
  lpt.set_vertex_color("white");
  lpt.set_label_color("off");
  lpt.set_label(m_us_name + "_tlpt");
  lpt.set_duration(600);
  string lpt_spec = lpt.get_spec();
  postMessage("VIEW_POINT", lpt_spec);
#endif
  
  applyHints(segl, m_hints, "turn");
  postMessage("VIEW_SEGLIST", segl.get_spec());
}

//-----------------------------------------------------------
// Procedure: postTurnPreview()

void BHV_LegRunZ::postTurnPreview(bool active)
{
  string label1 = m_us_name + "_pview1";
  string label2 = m_us_name + "_pview2";
  string label3 = m_us_name + "_pview3";
  
  XYSegList segl1(label1);
  XYSegList segl2(label2);
  XYSegList segl3(label3);

  if(!active) {
    postMessage("VIEW_SEGLIST", segl1.get_spec_inactive());
    postMessage("VIEW_SEGLIST", segl2.get_spec_inactive());
    postMessage("VIEW_SEGLIST", segl3.get_spec_inactive());
    return;
  }
  
  segl1 = m_legrun.initTurnPoints1();
  segl2 = m_legrun.initTurnPoints2();
  segl3 = XYSegList(m_legrun.getPoint1(), m_legrun.getPoint2());
  segl1.set_label(label1);
  segl2.set_label(label2);
  segl3.set_label(label3);
  
  if(segl1.size() != 0) {
    applyHints(segl1, m_hints, "turn");
    postMessage("VIEW_SEGLIST", segl1.get_spec());
  }
  
  if(segl2.size() != 0) {
    applyHints(segl2, m_hints, "turn");
    postMessage("VIEW_SEGLIST", segl2.get_spec());
  }

  if(segl3.size() != 0) {
    applyHints(segl3, m_hints, "legpt");
    postMessage("VIEW_SEGLIST", segl3.get_spec());
  }

  bool turn_pt_change = false;
  if(segl1.size() > 1) {
    double dist = segl1.length() * 0.42;
    XYPoint pt1 = segl1.get_dist_point(dist);
    if(distPointToPoint(pt1, m_turn_pt1) > 1) {
      m_turn_pt1 = pt1;
      turn_pt_change = true;
    }
  }
  if(segl2.size() > 1) {
    double dist = segl1.length() * 0.42;
    XYPoint pt2 = segl2.get_dist_point(dist);
    if(distPointToPoint(pt2, m_turn_pt2) > 1) {
      m_turn_pt2 = pt2;
      turn_pt_change = true;
    }
  }
  if(turn_pt_change)
    postTurnPoints();
  
  m_preview_pending = false;
}

//-----------------------------------------------------------
// Procedure: postSteerPoints()

void BHV_LegRunZ::postSteerPoints(bool active)
{
  m_trackpt.set_label(m_us_name + "'s track-point");
  m_nextpt.set_label(m_us_name + "'s next waypoint");
  applyHints(m_nextpt, m_hints, "nextpt");
  applyHints(m_trackpt, m_hints, "nextpt");
  string np_spec = m_nextpt.get_spec_inactive();
  string tp_spec = m_trackpt.get_spec_inactive();
  
  if(active) {
    np_spec = m_nextpt.get_spec();
    if(distPointToPoint(m_nextpt, m_trackpt) > 5)
      tp_spec = m_trackpt.get_spec();
  }
  
  postMessage("VIEW_POINT", tp_spec, "trk");
  postMessage("VIEW_POINT", np_spec, "wpt");
  
}

//-----------------------------------------------------------
// Procedure: postLegPoints()

void BHV_LegRunZ::postLegPoints(bool active)
{
  XYPoint p1 = m_legrun.getPoint1();
  XYPoint p2 = m_legrun.getPoint2();
  p1.set_label(getDescriptor() + "p1");
  p2.set_label(getDescriptor() + "p2");

  string spec1 = p1.get_spec_inactive();
  string spec2 = p2.get_spec_inactive();
  
  if(active) {
    applyHints(p1, m_hints, "legpt");
    applyHints(p2, m_hints, "legpt");
    // Draw P1 a bit bigger to distinguish the two.
    int vsize = p1.get_vertex_size();
    p1.set_vertex_size(vsize * 1.5);
    spec1 = p1.get_spec();
    spec2 = p2.get_spec();
  }

  postMessage("VIEW_POINT", p1.get_spec(), "p1");
  postMessage("VIEW_POINT", p2.get_spec(), "p2");
}

//-----------------------------------------------------------
// Procedure: eraseAllViewables()

void BHV_LegRunZ::eraseAllViewables()
{
  postLegSegList(false);
  postTurnSegList(false);
  postSteerPoints(false); 
  postLegPoints(false); 
  postTurnPreview(false); 
  postGrpAvgPoint(false); 
  postTurnPoints(false); 
}


//-----------------------------------------------------------
// Procedure: handleConfigLegSpeed()

bool BHV_LegRunZ::handleConfigLegSpeed(string str)
{
  str = tolower(removeWhite(str));
  
  // Part 1: Handle clearing the leg_spds vector
  if(str == "clear") {
    m_leg_spds.clear();
    m_leg_spds_ix = 0;
    return(true);
  }
  
  // Part 2: Handle resetting the speeds vector
  if(str == "reset") {
    m_leg_spds_ix = 0;
    return(true);
  }
  
  // Part 3: Handle replacing the speeds vector
  if(strBegins(str, "replace,")) {
    m_leg_spds.clear();
    m_leg_spds_ix = 0;
    biteString(str, ',');
  }
  
  double max_spd = m_domain.getVarHigh("speed");

  vector<double> new_leg_spds;
  
  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {

    if(strContains(svector[i], ":")) {
      string amt_str = biteString(svector[i], ':');
      string spd_str = svector[i];
      if(!isNumber(amt_str) || !isNumber(spd_str))
	return(false);
      int ival = atoi(amt_str.c_str());
      if(ival <= 0)
	return(false);
      double spd = atof(spd_str.c_str());
      if((spd < 0) || (spd > max_spd))
	return(false);
      for(int j=0; j<ival; j++)
	new_leg_spds.push_back(spd);
    }
    else {
      string spd_str = svector[i];
      if(!isNumber(spd_str))
	return(false);
      double spd = atof(spd_str.c_str());
      if((spd < 0) || (spd > max_spd))
	return(false);
      new_leg_spds.push_back(spd);
    }	
  }

  for(unsigned int i=0; i<new_leg_spds.size(); i++) {
    m_leg_spds.push_back(new_leg_spds[i]);
  }
  return(true);
}


//-----------------------------------------------------------
// Procedure: handleConfigCoord()
//     Notes: By default m_coord="" which indicates this behavior
//            will not try to coordinate the timing with other vehicles.
//            If m_coord is set to a non-empty string, this means
//            coordination is enabled. The value of this string
//            determines to which other vehicles coordination info
//            should be shared. If set to "true" or "all", info will
//            be set to all other vehicles in the field. If set to
//            "group=blue" or "blue", it will be shared with other
//            vehicles in the "blue" group. 

bool BHV_LegRunZ::handleConfigCoord(string val)
{
  // Since val could be a group name, which could have essentially an
  // unlimited set of possible names, the only thing we check for is
  // that there is no white space.
  if(strContainsWhite(val))
    return(false);

  // The setting "all" and "true" are equivalent, and means
  // coordination info will be share with all other vehicles
  // regardless of group name.  
  val = tolower(val);
  if(val == "true")
    m_coord = "all";
  else if(val == "false")
    m_coord = "";
  else
    m_coord = val;
  return(true);
}

//-----------------------------------------------------------
// Procedure: advanceModePending()

void BHV_LegRunZ::advanceModePending()
{
  if(m_mode == "leg1")
    m_mode_pending = "turn1";
  else if(m_mode == "turn1")
    m_mode_pending = "leg2";
  else if(m_mode == "leg2")
    m_mode_pending = "turn2";
  else if(m_mode == "turn2")
    m_mode_pending = "leg1";
  else if(m_mode == "leg1a")
    m_mode_pending = "leg2";
  else if(m_mode == "leg2a")
    m_mode_pending = "leg1";
}

//-----------------------------------------------------------
// Procedure: postConfigStatus()

void BHV_LegRunZ::postConfigStatus()
{
  string str = "type=BHV_LegRunZ,name=" + m_descriptor;

  //str += ",points=" + m_wpteng_legs.getPointsStr();
  str += ",speed=" + doubleToStringX(m_cruise_spd,1);

  str += ",currix=" + intToString(m_wpteng_legs.getCurrIndex());

  //bool repeats_endless = m_wpteng_legs.getRepeatsEndless();
  //if(repeats_endless)
  //  str += ",repeats=forever";
  //else {
  //  unsigned int repeats = m_wpteng_legs.getRepeats();
  //  str += ",repeats=" + uintToString(repeats);
  // }

  double capture_radius = m_wpteng_legs.getCaptureRadius();
  double slip_radius = m_wpteng_legs.getSlipRadius();
  
  str += ",capture_radius=" + doubleToStringX(capture_radius, 1);
  str += ",slip_radius=" + doubleToStringX(slip_radius, 1);

  bool using_capture_line = m_wpteng_legs.usingCaptureLine();
  if(using_capture_line) {
    if((capture_radius == 0) && (slip_radius == 0))
      str += ",capture_line=absolute";
    else
      str += ",capture_line=" + boolToString(using_capture_line);
  }
  
  str += ",patience=" + doubleToStringX(m_patience, 2);

  postRepeatableMessage("BHV_SETTINGS", str);
}

//-----------------------------------------------------------
// Procedure: expandMacros()

string BHV_LegRunZ::expandMacros(string sdata)
{
  // =======================================================
  // First expand the macros defined at the superclass level
  // =======================================================
  sdata = IvPBehavior::expandMacros(sdata);

  // =======================================================
  // Expand configuration parameters
  // =======================================================
  sdata = macroExpand(sdata, "SPEED", m_cruise_spd);
  
  // =======================================================
  // Expand Behavior State
  // =======================================================
  sdata = macroExpand(sdata, "NI", m_wpteng_legs.getCurrIndex());
  sdata = macroExpand(sdata, "NX", m_wpteng_legs.getPointX());
  sdata = macroExpand(sdata," NY", m_wpteng_legs.getPointY());

  sdata = macroExpand(sdata, "CYCLES", m_wpteng_legs.getCycleCount());
  sdata = macroExpand(sdata, "CYCREM", m_wpteng_legs.repeatsRemaining());
  sdata = macroExpand(sdata, "WPTS_HIT", m_wpteng_legs.getTotalHits());
  sdata = macroExpand(sdata, "MODE", m_mode);

  double dist_to_next_wpt = m_wpteng_legs.distToNextWpt(m_osx, m_osy);
  sdata = macroExpand(sdata, "DIST_NP", dist_to_next_wpt);
  double dist_to_prev_wpt = m_wpteng_legs.distToPrevWpt(m_osx, m_osy);
  sdata = macroExpand(sdata, "DIST_PP", dist_to_prev_wpt);
  double pct_to_next_wpt = m_wpteng_legs.pctToNextWpt(m_osx, m_osy);
  sdata = macroExpand(sdata, "PCT_NP", pct_to_next_wpt);

  double leg_odo = m_odometer.getTotalDist();
  sdata = macroExpand(sdata, "LEG_ODO", leg_odo);
  unsigned int leg_id = m_leg_count1 + m_leg_count2;
  sdata = macroExpand(sdata, "LEG_ID", leg_id);

  unsigned int wraps = m_wrap_detector.getWraps();
  sdata = macroExpand(sdata, "WRAPS", wraps);
  
  return(sdata);
}


//-----------------------------------------------------------
// Procedure: postLengthSpecs()

void BHV_LegRunZ::postLengthSpecs(bool force)
{
  // If no inter-vehicle coordination, don't share length
  // specs to other vehicles.
  if(m_coord == "")
    return;
  
  if(force)
    resetRegulatedMessage("LR_DIMS");
  
  double td1 = m_legrun.getTurn1Len();
  double td2 = m_legrun.getTurn2Len();
  double leg = m_legrun.getLegLen();

  string msg = "vname=" + m_us_name;
  msg += ",td1=" + doubleToStringX(td1,2);
  msg += ",td2=" + doubleToStringX(td2,2);
  msg += ",leg=" + doubleToStringX(leg,2);

  postOffboardMessage(m_coord, "LR_DIMS", msg);
}

//-----------------------------------------------------------
// Procedure: postPositionStatus()
//      Note: First the distance from current os position from the
//            beginning of legrun is calculated.
//            Beginning is at P1 heading to P2.
//            End is upon completion of turn1 arriving back at P1.
//           
//      Note: At the beginning the degrees is zero, increasing up
//            to 360 approaching the end. 
//
//   Example: vname=abe, rd=245, osv=2.3
//
//-----------------------------------------------------------------
//                        <--mode=leg1                            |
//                                                                |
// mode=     /--------o--------------------o---------\    ^       |
// turn1     | t1  |  p1                  p2   | t2  |    |       |
//   |       \    /                             \    /  mode=     |
//   V        ---/         mode=leg2-->          \---   turn2     |
//-----------------------------------------------------------------

// Somewhere in here we may want to add logic to account for the fact
// when the capture radius is N, the vehicle will enter the next mode
// but the Leg vertex is still in front of the vehicle. Could perhaps
// mitigate by setting the capture radius ~0 and rely solely on the
// slip radius.

void BHV_LegRunZ::postPositionStatus()
{
  XYPoint p1 = m_legrun.getPoint1();
  XYPoint p2 = m_legrun.getPoint2();

  double  leg = m_legrun.getLegLen();
  double  td1 = m_legrun.getTurn1Len(); 
  double  td2 = m_legrun.getTurn2Len(); 

  double  glr_td1 = m_ring_master.getGrpTurn1Len();
  double  glr_td2 = m_ring_master.getGrpTurn2Len();
  double  glr_tot = m_ring_master.getGrpTotalLen();
  
  double  glr_dist = 0; // remaining dist/length, given mode/ospos

  // Sanity checks to be doubly sure not to divide by zero
  if((td1==0) || (td2==0) || (glr_tot==0))
    return;
  
  if(m_mode == "leg2") {
    double dist_to_p2 = hypot(m_osx - p2.x(), m_osy - p2.y());
    double dist_from_p1 = leg - dist_to_p2;
    if(dist_from_p1 < 0)
      dist_from_p1 = 0;
    glr_dist = dist_from_p1;
  }
  else if(m_mode == "turn2") {
    double dist_from_beg = m_wpteng_turn.distFromBeg(m_osx, m_osy);
    glr_dist = leg + dist_from_beg * (glr_td2 / td2);
  }
  else if(m_mode == "leg1") {
    // Calculate the distance from p2 in terms of distance to p1.
    // In other words, if the vehicle is off the line, that offline
    // distance does not count
    double dist_to_p1 = hypot(m_osx - p1.x(), m_osy - p1.y());
    double dist_from_p2 = leg - dist_to_p1;
    if(dist_from_p2 < 0)
      dist_from_p2 = 0;
    //double dist_from_p2 = hypot(m_osx - p2.x(), m_osy - p2.y());
    glr_dist = dist_from_p2 + glr_td2 + leg;
  }
  else if(m_mode == "turn1") {
    double dist_from_beg = m_wpteng_turn.distFromBeg(m_osx, m_osy);
    glr_dist = dist_from_beg * (glr_td1 / td1);
    glr_dist += (2*leg) + glr_td2;
  }
  
  else {
    postWMessage("Undefined mode:" + m_mode);
    return;
  }
  
  double degs = (glr_dist / glr_tot) * 360;
  double degs_per_sec = (360/glr_tot) * m_osv;
  double utc = getBufferCurrTime();
  m_ring_master.updatePosCN("ownship", degs_per_sec, degs, utc);
  //m_ring_master.updateCenter();

  string rd = doubleToStringX(degs,2);
  string osv = doubleToStringX(degs_per_sec,2);
  
  string report = "vname=" + m_us_name + ",rd=" + rd + ",osv=" + osv;


  postMessage("LR_OS_DEGS", degs);
  postOffboardMessage(m_coord, "LR_REMX_DIST", report);  
}

//-----------------------------------------------------------
// Procedure: setTurnCoordSpd()
//
//-----------------------------------------------------------------
//                        <--mode=leg1                            |
//                                                                |
// mode=     /--------o--------------------o---------\    ^       |
// turn1     | t1  |  p1                  p2   | t2  |    |       |
//   |       \    /                             \    /  mode=     |
//   V        ---/         mode=leg2-->          \---   turn2     |
//-----------------------------------------------------------------


void BHV_LegRunZ::setTurnCoordSpd(double gpa)
{
 // By default coord turn spd is disabled 
  m_turn_coord_spd = -1;
  // If coord disabled just return with turn_spd_coord at -1
  if(m_coord == "")
    return;
  
  double raw_dist = 0;
  double glr_dist = 0;

  double raw_degs = m_ring_master.getOwnDegs();
  
  //cout << "cc: gpa=" << gpa << ", raw_degs=" << raw_degs << endl;
  if(m_mode == "leg2") {
    //raw_dist = rawDistToP2(raw_degs);
    //raw_dist = rawDistToP2(gpa);
    //cout << "cc: leg2: rawDistToP2:+++" << endl;
    raw_dist = glrDistToP2(raw_degs);

    //glr_dist = rawDistToP2(gpa);
    //cout << "cc: leg2: glrDistToP2:+++" << endl;
    glr_dist = glrDistToP2(gpa);
    //cout << "cc: M:leg2 ToP2, rd=" << raw_dist << ", gd=" << glr_dist << endl;
  }
  else if(m_mode == "turn2") {
    //raw_dist = rawDistToP1(raw_degs);
    //raw_dist = rawDistToP1(gpa);
    //cout << "cc: turn2: rawDistToP1:+++" << endl;
    raw_dist = glrDistToP1(raw_degs);

    //glr_dist = rawDistToP1(gpa);
    //cout << "cc: turn2: glrDistToP1:+++" << endl;
    glr_dist = glrDistToP1(gpa);
    cout << "cc: M:turn2 ToP1, rd=" << raw_dist << ", gd=" << glr_dist << endl;
  }
  else if(m_mode == "leg1") {
    //raw_dist = rawDistToP1(raw_degs);
    //raw_dist = rawDistToP1(gpa);
    //cout << "cc: leg1: rawDistToP1:+++" << endl;
    raw_dist = glrDistToP1(raw_degs);

    //glr_dist = rawDistToP1(gpa);
    //cout << "cc: leg1: glrDistToP1:+++" << endl;
    glr_dist = glrDistToP1(gpa);
    //cout << "cc: M:leg1 ToP1, rd=" << raw_dist << ", gd=" << glr_dist << endl;
  }
  else if(m_mode == "turn1") {
    //raw_dist = rawDistToP2(raw_degs);
    //raw_dist = rawDistToP2(gpa);
    //cout << "cc: turn1: rawDistToP2:+++" << endl;
    raw_dist = glrDistToP2(raw_degs);

    //glr_dist = rawDistToP2(gpa);
    //cout << "cc:turn1: glrDistToP2:+++" << endl;
    glr_dist = glrDistToP2(gpa);
    //cout << "cc: M:turn1 ToP2, rd=" << raw_dist << ", gd=" << glr_dist << endl;
  }

  postMessage("LR_RAW_DEGS", raw_degs);
  postMessage("LR_GLR_DEGS", gpa);
  postMessage("LR_RAW_DIST", raw_dist);
  postMessage("LR_GLR_DIST", glr_dist);
  
  // A positive offset: The grp is ahead of us, need to speed up
  // A negative offset: The grp is behind us, need to slow down
  
  double offset = raw_dist - glr_dist;
  //cout << "cc: offset(1): " << offset << endl;

  double grp_total_len = m_ring_master.getGrpTotalLen();
  if(offset > (grp_total_len/2))
    offset -= grp_total_len;
  if(offset < (-grp_total_len/2))
    offset += grp_total_len;
  //cout << "cc: offset(2): " << offset << endl;

  m_turn_coord_spd = m_spd_modulator.getSpdFromPolicy(offset, m_cruise_spd);
  //cout << "cc: cruise_spd: " << m_cruise_spd << endl;
  //cout << "cc: turn_coor_spd: " << m_turn_coord_spd << endl;
  postMessage("LR_TC_SPD", m_turn_coord_spd);
  postMessage("LR_OFFSET", offset);
  
}

//-----------------------------------------------------------
// Procedure: postGrpAvgPoint()

void BHV_LegRunZ::postGrpAvgPoint(double gpa, bool active)
{
  XYPoint dpt;
  dpt.set_label(m_us_name + "_dpt");
  string dpt_spec = dpt.get_spec_inactive();

  if(active) {
    double dist_from_beg = rawDistFromP1(gpa);
    dpt = m_legrun.getDistPt(dist_from_beg);
    dpt.set_label(m_us_name + "_dpt");

    postMessage("LR_GPA", gpa);
  
    dpt.set_vertex_size(12);
    dpt.set_vertex_color("magenta");
    dpt.set_label_color("off");
    dpt_spec = dpt.get_spec();

  }

  // Reduce the frequency of posting visual points when the
  // points are not that far apart. 0.6 m threshold seems good.
  if(dpt.active() && m_prev_dpt.active())
    if(distPointToPoint(dpt, m_prev_dpt) < 0.6)
      return;
  
  m_prev_dpt = dpt;

  postMessage("VIEW_POINT", dpt_spec);
}

//-----------------------------------------------------------
// Procedure: postTurnPoints()

void BHV_LegRunZ::postTurnPoints(bool active)
{
  string tp1_spec = m_turn_pt1.get_spec_inactive();
  string tp2_spec = m_turn_pt2.get_spec_inactive();
  if(active) {
    m_turn_pt1.set_vertex_size(8);
    m_turn_pt1.set_vertex_color("limegreen");
    m_turn_pt1.set_label_color("off");
    m_turn_pt1.set_label(m_us_name + "_fpt1");
    m_turn_pt1.set_duration(600);
    
    m_turn_pt2.set_vertex_size(8);
    m_turn_pt2.set_vertex_color("limegreen");
    m_turn_pt2.set_label_color("off");
    m_turn_pt2.set_label(m_us_name + "_fpt2");
    m_turn_pt2.set_duration(600);
    
    tp1_spec = m_turn_pt1.get_spec();
    tp2_spec = m_turn_pt2.get_spec();
  }
  
  postMessage("VIEW_POINT", tp1_spec);
  postMessage("VIEW_POINT", tp2_spec);
}

//-----------------------------------------------------------
// Procedure: rawDistToP1()

double BHV_LegRunZ::rawDistToP1(double gpa)
{
  double leg_len   = m_legrun.getLegLen();
  double turn1_len = m_legrun.getTurn1Len();
  double turn2_len = m_legrun.getTurn2Len();

  return(distToP1(gpa, leg_len, turn1_len, turn2_len));
}

//-----------------------------------------------------------
// Procedure: rawDistToP2()

double BHV_LegRunZ::rawDistToP2(double gpa)
{
  double leg_len   = m_legrun.getLegLen();
  double turn1_len = m_legrun.getTurn1Len();
  double turn2_len = m_legrun.getTurn2Len();

  return(distToP2(gpa, leg_len, turn1_len, turn2_len));
}

//-----------------------------------------------------------
// Procedure: glrDistToP1()

double BHV_LegRunZ::glrDistToP1(double gpa)
{
  double glr_td1 = m_ring_master.getGrpTurn1Len();
  double glr_td2 = m_ring_master.getGrpTurn2Len();
  double leg_len = m_legrun.getLegLen();

  return(distToP1(gpa, leg_len, glr_td1, glr_td2));
}

//-----------------------------------------------------------
// Procedure: glrDistToP2()

double BHV_LegRunZ::glrDistToP2(double gpa)
{
  double glr_td1 = m_ring_master.getGrpTurn1Len();
  double glr_td2 = m_ring_master.getGrpTurn2Len();
  double leg_len = m_legrun.getLegLen();

  return(distToP2(gpa, leg_len, glr_td1, glr_td2));
}

//-----------------------------------------------------------
// Procedure: distToP1()
//      Note: Calculates the dist to P1 *after* turn1. 
//
//-----------------------------------------------------------
//                       <--mode=leg1                       |
//                                                          |
//  ^      /--------o------------------o---------\    ^     |
//  |      | t1  |  p1                p2   | t2  |    |     |
// mode=   \    /                           \    /  mode=   |
// turn1    ---/         mode=leg2-->        \---   turn2   |
//                                                          |
//-----------------------------------------------------------
//
// mode=leg2  --> dist_from_p1 + turn2 + leg
// mode=turn2 --> dist_from_p2 + leg
// mode=leg1  --> dist_to_p1
// mode=turn2 --> dist_to_p1 + leg + turn2 + leg

// Returns Pos number if closest direction is forward
//         Neg number if closest direction is behind

double BHV_LegRunZ::distToP1(double gpa, double leg_len,
			     double turn1_len, double turn2_len)
{
  // Sanity checks
  if((gpa < 0) || (gpa > 360))
    return(0);
  if((leg_len < 0) || (turn1_len < 0) || (turn2_len < 0))
    return(0);

  double total_len = (2*leg_len) + turn1_len + turn2_len;

  //cout << "cck: --- gpa=" << gpa << endl;
  //cout << "cck: total_len:" << total_len << endl; 

  //cout << "cck: leg=" << leg_len << endl;
  //cout << "cck: t1d=" << turn1_len << endl;
  //cout << "cck: t2d=" << turn2_len << endl;

  double thresh = ((2*leg_len + turn2_len) / total_len) * 360;
  //cout << "cck: gpa_thresh:" << thresh << endl; 

  double dist_to_p1 = 0;
  if(gpa < thresh) {
    dist_to_p1 = (thresh-gpa)/360 * total_len;
    //cout << "cck: dist_f:" << dist_to_p1 << endl;
  }
  else {
    dist_to_p1 = total_len - ((gpa-thresh)/360 * total_len);
    //cout << "cck: dist_w:" << dist_to_p1 << endl; 
  }

#if 0
  double dist_from_beg = (gpa / 360) * total_len;
  double dist_to_end   = total_len - dist_from_beg;
  double dist_to_p1  = dist_to_end + (2*leg_len) + turn2_len;
  if(dist_to_p1 > total_len)
    dist_to_p1 -= total_len;
  //cout << "cck: dist_from_beg:" << dist_from_beg << endl; 
  //cout << "cck: dist_to_end(1):" << dist_to_end << endl; 
#endif

  if(dist_to_p1 < (total_len/2))
    return(dist_to_p1);
  
  double mod_dist_to_p1 = dist_to_p1 - total_len;
  
  //cout << "cck: mod_dist_to_end:" << mod_dist_to_p1 << endl; 

  return(mod_dist_to_p1);
}


//-----------------------------------------------------------
// Procedure: distToP2()
//      Note: Calculates the dist to P2 *before* turn2. 
//
//-----------------------------------------------------------
//                       <--mode=leg1                       |
//                                                          |
//  ^      /--------o------------------o---------\    ^     |
//  |      | t1  |  p1                p2   | t2  |    |     |
// mode=   \    /                           \    /  mode=   |
// turn1    ---/         mode=leg2-->        \---   turn2   |
//                                                          |
//-----------------------------------------------------------
//
// Returns Pos number if closest direction is forward
//         Neg number if closest direction is behind

double BHV_LegRunZ::distToP2(double gpa, double leg_len,
			     double turn1_len, double turn2_len)
{
  // Sanity checks
  if((gpa < 0) || (gpa > 360))
    return(0);
  if((leg_len < 0) || (turn1_len < 0) || (turn2_len < 0))
    return(0);

  double total_len = (2*leg_len) + turn1_len + turn2_len;

  double dist_from_beg = (gpa / 360) * total_len;
  double dist_to_end = total_len - dist_from_beg;
  double dist_to_p2  = dist_to_end + leg_len;
  if(dist_to_p2 > total_len)
    dist_to_p2 -= total_len;

  if(dist_to_p2 < (total_len/2))
    return(dist_to_p2);
  
  //double mod_dist_to_p2 = -(dist_to_p2 - (total_len/2));
  double mod_dist_to_p2 = dist_to_p2 - total_len;
  
  return(mod_dist_to_p2);
}


//-----------------------------------------------------------
// Procedure: rawDistFromP1
//
//-----------------------------------------------------------
//                        <--mode=leg1                      |
//                                                          |
// mode=    /--------o------------------o---------\    ^    |
// turn1    | t1  |  p1                p2   | t2  |    |    |
//   |      \    /                           \    /  mode=  |
//   V       ---/         mode=leg2-->        \---   turn2  |
//                                                          |
//-----------------------------------------------------------
//
// Returns Always a postive number

double BHV_LegRunZ::rawDistFromP1(double gpa)
{
  // Sanity checks
  if((gpa < 0) || (gpa > 360))
    return(0);

  double glr_td1 = m_ring_master.getGrpTurn1Len();
  double glr_td2 = m_ring_master.getGrpTurn2Len();
  double glr_tot = m_ring_master.getGrpTotalLen();
  double glr_len = (gpa /360) * glr_tot;
  
  double td1 = m_legrun.getTurn1Len();
  double td2 = m_legrun.getTurn2Len();
  double leg = m_legrun.getLegLen();

  double ra1 = td1 / glr_td1;
  double ra2 = td2 / glr_td2;

  if(glr_len < leg)
    return(glr_len);

  glr_len -= leg;
  if(glr_len < glr_td2)
    return(leg + (glr_len * ra2));

  glr_len -= glr_td2;
  if(glr_len < leg)
    return(leg + td2 + glr_len);

  glr_len -= leg;
  if(glr_len < glr_td1)
    return(leg + td2 + leg + (glr_len * ra1));

  return(0);  
}
