/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_AndersonTurn.cpp                                 */
/*    DATE: July 26th 2020                                       */
/*    DATE: Oct 21st, 2021 Major restructure                     */
/*****************************************************************/

#include <iostream>
#include <cmath> 
#include <cstdlib>
#include "BHV_AndersonTurn.h"
#include "MBUtils.h"
#include "MacroUtils.h"
#include "GeomUtils.h"
#include "AngleUtils.h"
#include "BuildUtils.h"
#include "ZAIC_PEAK.h"
#include "ZAIC_SPD.h"
#include "OF_Coupler.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_AndersonTurn::BHV_AndersonTurn(IvPDomain gdomain) : 
  IvPBehavior(gdomain)
{
  this->setParam("descriptor", "bhv_aturn");

  m_domain = subDomain(m_domain, "course,speed");

  // Init configuration vars
  m_peakwidth   = 0;
  m_basewidth   = 0.2;
  m_summitdelta = 0;

  m_default_capture_radius = 4;
  m_default_turn_radius = 15;
  m_default_turn_points = 12;
  m_default_turn_thresh = 315;
  
  m_capture_radius = m_default_capture_radius;
  m_turn_radius = m_default_turn_radius;
  m_turn_points = m_default_turn_points;
  m_turn_thresh = m_default_turn_thresh;
  
  resetStateVars();
}

//-----------------------------------------------------------
// Procedure: resetStateVars()

void BHV_AndersonTurn::resetStateVars()
{
  // State vars
  m_osx = 0;
  m_osy = 0;

  m_wptx = 0;
  m_wpty = 0;
  m_wix  = 0;
  m_kx = 0;
  m_ky = 0;
  
  m_prev_hdg = 0;
  m_curr_hdg = 0;
  m_curr_spd = 0;
  m_mark_hdg = 0;
  m_mark_spd = 0;
  m_all_turn = 0;
  m_state = "off";  
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_AndersonTurn::setParam(string param, string val) 
{
  if(IvPBehavior::setParam(param, val))
    return(true);

  bool handled = false;
  if(param == "engage_var")
    handled = setNonWhiteVarOnString(m_engage_var, val);
  else if((param == "capture_radius") || (param == "caprad"))
    handled = setPosDoubleOnString(m_capture_radius, val);
  else if((param == "turn_radius") || (param == "turnrad"))
    handled = setPosDoubleOnString(m_turn_radius, val);
  else if((param == "turn_thresh") || (param == "thresh"))
    handled = setDoubleStrictRngOnString(m_turn_thresh, val, 0, 360);
  else if(param == "turn_pts")
    handled = setPosUIntOnString(m_turn_points, val);
  else if(param == "peakwidth")
    handled = setNonNegDoubleOnString(m_peakwidth, val);
  else if(param == "basewidth")
    handled = setNonNegDoubleOnString(m_basewidth, val);
  else if((param == "summitdelta") && isNumber(val)) {
    double dval = atof(val.c_str());
    m_summitdelta = vclip(dval, 0, 100);
    handled = true;
  }
  else if(param == "default_capture_radius")
    handled = setPosDoubleOnString(m_default_capture_radius, val);
  else if(param == "default_turn_radius")
    handled = setPosDoubleOnString(m_default_turn_radius, val);
  else if(param == "default_turn_points")
    handled = setPosUIntOnString(m_default_turn_points, val);
  else if(param == "default_turn_thresh")
    handled = setDoubleStrictRngOnString(m_default_turn_thresh, val, 0, 360);
  
  return(handled);
}

//-----------------------------------------------------------
// Procedure: onSetParamComplete()

void BHV_AndersonTurn::onSetParamComplete()
{
  if(m_engage_var != "")
    addInfoVars(m_engage_var);
  
  postConfigStatus();
}


//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_AndersonTurn::onRunState() 
{
  updateInfoIn();
  checkForEngageMsg();

  setMarkWaypt();

  if(m_state == "off")
    return(0);

  
  IvPFunction *ipf = buildOF();
  if(ipf)
    ipf->setPWT(m_priority_wt);
  else 
    postEMessage("Unable to generate IvP function");

  return(ipf);
}

//-----------------------------------------------------------
// Procedure: onIdleToRunState()

void BHV_AndersonTurn::onIdleToRunState() 
{
  resetStateVars();
}

//-----------------------------------------------------------
// Procedure: onRunToIdleState()

void BHV_AndersonTurn::onRunToIdleState() 
{
  handleComplete();
}

//-----------------------------------------------------------
// Procedure: updateInfoIn()
//   Purpose: Update relevant to the behavior from the info_buffer.
//   Returns: true if no relevant info is missing from the info_buffer.
//            false otherwise.

bool BHV_AndersonTurn::updateInfoIn()
{
  // =================================================
  // Part 1: Update ownship current heading and speed
  // =================================================
  if(!getBufferDoubleValX("NAV_X", m_osx))
    return(false);
  if(!getBufferDoubleValX("NAV_Y", m_osy))
    return(false);
  if(!getBufferDoubleValX("NAV_SPEED", m_curr_spd))
    return(false);

  m_prev_hdg = m_curr_hdg;
  if(!getBufferDoubleValX("NAV_HEADING", m_curr_hdg))
    return(false);
  
  // =================================================
  // Part 2: If turning, determine total progress
  // For Port turns, m_all_turn must reach -thresh.
  // For Star turns, m_all_turn must reach +thresh.
  // =================================================
  if(m_state == "off")
    return(true);
  
  double hdg_delta = angleDiff(m_curr_hdg, m_prev_hdg);
  m_all_turn += hdg_delta;

  //cout << "hdg_delta: " << doubleToString(hdg_delta) << endl;
  //cout << "all_turn:  " << doubleToString(m_all_turn) << endl;
  
  if(m_all_turn > m_turn_thresh) 
    handleComplete();
  
  return(true);
}

//-----------------------------------------------------------
// Procedure: checkForEngageMsg()
//   Example: ENGAGE=port,turnrad=15,caprad=5,turnpts=16

void BHV_AndersonTurn::checkForEngageMsg()
{
  bool new_engage_msg = getBufferVarUpdated(m_engage_var);
  if(!new_engage_msg)
    return;

  string new_state;

  string msg = tolower(getBufferStringVal(m_engage_var));
  vector<string> svector = parseString(msg, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string orig  = svector[i];
    string param = biteStringX(svector[i], '=');
    string value = svector[i];

    if(param == "starboard")
      param = "star";
    bool handled = false;
    if((param=="port") || (param=="star") || (param=="off")) {
      new_state = param;
      handled = true;
    }
    else if(param == "turnrad")
      handled = setPosDoubleOnString(m_turn_radius, value); 
    else if(param == "caprad")
      handled = setPosDoubleOnString(m_capture_radius, value);
    else if(param == "thresh")
      handled = setDoubleStrictRngOnString(m_turn_thresh, value, 0, 360);
    else if(param == "turnpts")
      handled = setPosUIntOnString(m_turn_points, value);

    if(!handled) {
      postWMessage("Bad Engage msg: " + orig);
      return;
    }
  }

  if(m_state == new_state)
    return;
  m_state = new_state;

  if(m_state == "off") {
    handleComplete();
    return;
  }

  m_prev_hdg = m_curr_hdg;
  m_mark_hdg = m_curr_hdg;
  m_mark_spd = m_curr_spd;
  m_all_turn = 0;
  
  genTurnPath();
  setMarkWaypt(true);
  drawTurnPath();
}


//-----------------------------------------------------------
// Procedure: buildOF()

IvPFunction *BHV_AndersonTurn::buildOF() 
{
  if(m_state == "off")
    return(0);

  double desired_hdg = angle360(m_curr_hdg + 18);
  if(m_state == "port")
    desired_hdg = angle360(m_curr_hdg - 25);

  desired_hdg = relAng(m_osx,m_osy, m_wptx,m_wpty);
  
  //=====================================================
  // Part 2: Build the Course ZAIC
  //=====================================================
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(desired_hdg);
  crs_zaic.setBaseWidth(m_basewidth);
  crs_zaic.setPeakWidth(m_peakwidth);
  crs_zaic.setSummitDelta(m_summitdelta);
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();
  if(!crs_ipf)
    postWMessage("Failure on the CRS ZAIC");
  postWMessage(crs_zaic.getWarnings());
  
  //=====================================================
  // Part 3: Build the Speed ZAIC
  //=====================================================
  ZAIC_SPD spd_zaic(m_domain, "speed");
  spd_zaic.setParams(m_mark_spd, 0.1, m_mark_spd+0.4, 70, 20);
  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction(); 
  if(!spd_ipf)
    postWMessage("Failure on the SPD ZAIC via ZAIC_SPD utility");
  postWMessage(spd_zaic.getWarnings());

  //=====================================================
  // Part 4: Couple the two compents
  //=====================================================
  double course_pct = 50;
  double speed_pct  = 50;
  
  OF_Coupler coupler;
  IvPFunction *ipf = coupler.couple(crs_ipf, spd_ipf, course_pct, speed_pct);
  if(!ipf)
    postWMessage("Failure on the CRS_SPD COUPLER");

  return(ipf);
}

//---------------------------------------------------------------
// Procedure: setMarkWaypt()

void BHV_AndersonTurn::setMarkWaypt(bool init)
{
  // Sanity checks
  if(m_turn_path.size() == 0)
    return;
  if((m_state != "star") && (m_state != "port"))
    return;
  
  // Special init case when turn is initiated, the mark point is
  // set to the ownship position.
  if(init) {
    m_kx = m_osx;
    m_ky = m_osy;
    m_wix = 0;
  }

  m_wptx = m_turn_path[m_wix].x();
  m_wpty = m_turn_path[m_wix].y();

  bool next = false;
  
  // Check 1: range
  double range = hypot(m_osx-m_wptx, m_osy-m_wpty);
  if(range < m_capture_radius)
    next = true;

  // Check 2: angle
  double ang = angleFromThreePoints(m_wptx,m_wpty, m_kx,m_ky, m_osx,m_osy);
  if(ang > 80)
    next = true;

  if(!next)
    return;

  m_wix++;
  if(m_wix >= m_turn_path.size()) {
    handleComplete();
    return;
  }

  m_wptx = m_turn_path[m_wix].x();
  m_wpty = m_turn_path[m_wix].y();
  m_kx = m_osx;
  m_ky = m_osy;

  if(init || next)
    drawTurnPath();  
}


//---------------------------------------------------------------
// Procedure: genTurnPathPort()

void BHV_AndersonTurn::genTurnPath()
{
  m_turn_path.clear();
  double delta = 360 / (double)(m_turn_points);

  double cx, cy;
  double ang1 = angle360(m_curr_hdg + 90);
  double ang2 = angle360(m_curr_hdg - 90);
  if(m_state == "port") {
    ang1 = angle360(m_curr_hdg - 90);
    ang2 = angle360(m_curr_hdg + 90);
  }
  projectPoint(ang1, m_turn_radius, m_osx, m_osy, cx, cy);
  
  for(unsigned int i=0; i<m_turn_points; i++) {
    double angi = ang2;
    if(m_state == "port") {
      angi -= (delta/2);
      angi -= ((double)(i)) * delta;
    }
    else {
      angi += (delta/2);
      angi += ((double)(i)) * delta;
    }
    angi = angle360(angi);
    
    double nx, ny;
    projectPoint(angi, m_turn_radius, cx,cy, nx,ny);
    XYPoint new_point(nx,ny);
    m_turn_path.push_back(new_point);
  }  
}


//---------------------------------------------------------------
// Procedure: handleComplete()

void BHV_AndersonTurn::handleComplete()
{
  m_state = "off";

  // Turn characteristics revert back to their defaults
  m_turn_thresh    = m_default_turn_thresh;
  m_capture_radius = m_default_capture_radius;
  m_turn_radius    = m_default_turn_radius;
  m_turn_points    = m_default_turn_points;

  eraseTurnPath();

  m_turn_path.clear();
  m_wix = 0;

  // Invoke behavior complete, endflags etc
  setComplete();
}


//---------------------------------------------------------------
// Procedure: drawTurnPath()

void BHV_AndersonTurn::drawTurnPath()
{
  for(unsigned int i=0; i<m_turn_path.size(); i++) {
    XYPoint point = m_turn_path[i];
    if(i<m_wix)
      point.set_vertex_color("gray35");
    else
      point.set_vertex_color("gray95");
    point.set_label_color("off");
    point.set_vertex_size(7);
    point.set_label("p_" + uintToString(i));
    string spec = point.get_spec();
    postMessage("VIEW_POINT", spec);
  }
}


//---------------------------------------------------------------
// Procedure: eraseTurnPath()

void BHV_AndersonTurn::eraseTurnPath()
{
  for(unsigned int i=0; i<m_turn_path.size(); i++) {
    XYPoint point = m_turn_path[i];
    point.set_label("p_" + uintToString(i));
    string spec = point.get_spec_inactive();
    postMessage("VIEW_POINT", spec);
  }
}


//---------------------------------------------------------------
// Procedure: headingDelta()
//   Purpose: o Return the change in heading with the proper sign.
//            o We assume the smallest heading delta represents the
//              the true change in heading. For example 10,12 is a
//              starboard turn of 2 degrees rather than a port turn
//              of 358 degrees.

double BHV_AndersonTurn::headingDelta(double ang1, double ang2)
{
  double ang_clock  = angleDiffClock(ang1, ang2);
  double ang_cclock = angleDiffCounterClock(ang1, ang2);
  if(ang_clock < ang_cclock)
    return(ang_clock);
  else
    return(-ang_cclock);
}


//---------------------------------------------------------------
// Procedure: angleDiffClock()
//   Purpose: Determine the difference in angle degrees between 
//            two given angles, starting from ang1, proceeding
//            clockwise to ang2.
//  Examples: angleDiffClock(350, 300) = 310

double BHV_AndersonTurn::angleDiffClock(double ang1, double ang2)
{
  ang1 = angle360(ang1);
  ang2 = angle360(ang2);
  double diff;
  if(ang2 >= ang1)
    diff = ang2 - ang1;
  else
    diff = ang2 - (ang1-360);

  return(diff);
}

//---------------------------------------------------------------
// Procedure: angleDiffCounterClock()
//   Purpose: Determine the difference in angle degrees between 
//            two given angles, starting from ang1, proceeding
//            counter clockwise to ang2.
//  Examples: angleDiffClock(350, 300) = 50
//            angleDiffClock(300, 350) = 310

double BHV_AndersonTurn::angleDiffCounterClock(double ang1, double ang2)
{
  ang1 = angle360(ang1);
  ang2 = angle360(ang2);
  double diff;
  if(ang1 >= ang2)
    diff = ang1 - ang2;
  else
    diff = ang1 - (ang2-360);

  return(diff);
}


//-----------------------------------------------------------
// Procedure: expandMacros()

string BHV_AndersonTurn::expandMacros(string sdata)
{
  // =======================================================
  // First expand the macros defined at the superclass level
  // =======================================================
  sdata = IvPBehavior::expandMacros(sdata);

  // =======================================================
  // Expand configuration parameters
  // =======================================================
  sdata = macroExpand(sdata, "TRAD", m_turn_radius);
  sdata = macroExpand(sdata, "MKHDG", m_mark_hdg);
  sdata = macroExpand(sdata, "MKSPD", m_mark_spd);
  
  // =======================================================
  // Expand Behavior State
  // =======================================================
  sdata = macroExpand(sdata, "TDEGS", m_all_turn);
  sdata = macroExpand(sdata, "TMODE", m_state);

  return(sdata);
}


