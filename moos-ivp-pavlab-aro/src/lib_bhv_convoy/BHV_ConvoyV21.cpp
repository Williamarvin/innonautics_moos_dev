/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_ConvoyV21.cpp                                    */
/*    DATE: April 4th, 2019                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/
 
#include <cmath>
#include <cstdlib>
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "BHV_ConvoyV21.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "ZAIC_PEAK.h"
#include "ZAIC_SPD.h"
#include "OF_Coupler.h"
#include "MacroUtils.h"
#include "VarDataPairUtils.h"
#include "ConvoyRecap.h"
#include "ConvoyStatRecap.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_ConvoyV21::BHV_ConvoyV21(IvPDomain gdomain) : 
  IvPContactBehavior(gdomain)
{
  this->setParam("descriptor", "convoy");
  
  m_domain = subDomain(m_domain, "course,speed");

  // ====================================================
  // Initialize State variables
  // ====================================================
  m_wptx    = 0;
  m_wpty    = 0;
  m_set_speed = 0;
    
  m_cnv_avg_2sec = 0;
  m_cnv_avg_5sec = 0;

  m_convoy_range = 1;

  m_post_recap_verbose = false;
  
  // ====================================================
  // Initialize State variables (Metrics)
  // ====================================================
  m_tail_range = -1;
  m_tail_angle = -1;
  m_marker_bng = -1;
  m_alignment  = -1;

  m_reached_markers = 0;
  m_dropped_markers = 0;

  m_recap_index = 0;
  m_stat_recap_index = 0;
  
  // ====================================================
  // Initialize Config variables
  // ====================================================
  m_capture_radius = 5;    // meters
  m_slip_radius    = 20;   // meters
  m_compression    = 0;    // [0,1]
  m_patience       = 50;   // [1,99]
  m_aft_patience   = false;

  m_max_speed = -1; // Neg val means no additional max speed
  
  m_hint_marker_color = "dodger_blue";
  m_hint_marker_label_color = "off";
  m_hint_marker_size  = 4;

  m_holding_policy = "zero";
  
  addInfoVars("NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING");
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_ConvoyV21::setParam(string param, string param_val) 
{
  if(IvPContactBehavior::setParam(param, param_val))
    return(true);
  
  bool handled = false;
  if((param == "nm_radius") || (param == "slip_radius"))
    handled = setNonNegDoubleOnString(m_slip_radius, param_val);
  else if((param == "radius") || (param == "capture_radius"))
    handled = setNonNegDoubleOnString(m_capture_radius, param_val);

  else if(param == "tail_length_max") 
    handled = m_marker_tail.setMaxTailLength(param_val);
  else if(param == "inter_mark_range") 
    handled = m_marker_tail.setInterMarkRange(param_val);

  else if((param == "full_stop_convoy_range") ||
	  (param == "slower_convoy_range")    ||
	  (param == "ideal_convoy_range")     ||
	  (param == "faster_convoy_range")    ||
	  (param == "full_lag_convoy_range")  || 
	  (param == "lag_speed_delta"))
    handled = m_spd_policy_base.setParam(param, param_val);

  else if(param == "visual_hints")
    return(handleParamVisualHints(param_val));
  else if(param == "compression")
    return(handleParamCompression(param_val));
  else if(param == "aft_patience")
    return(setBooleanOnString(m_aft_patience, param_val));
  else if(param == "holding_policy")
    return(handleConfigHoldingPolicy(param_val));

  else if(param == "marker_flag")
    return(addFlagOnString(m_marker_flags, param_val));
  else if(param == "convoy_flag")
    return(addFlagOnString(m_convoy_flags, param_val));

  else if(param == "post_recap_verbose")
    handled = setBooleanOnString(m_post_recap_verbose, param_val);
  
  else if(param == "max_speed")
    handled = setDoubleOnString(m_max_speed, param_val);
  
  else if((param == "patience") && isNumber(param_val)) {
    double dval = atof(param_val.c_str());
    if((dval < 1) || (dval > 99))
      return(false);
    m_patience = dval;
    return(true);
  }
  
  return(handled);
}

//-----------------------------------------------------------
// Procedure: handleConfigHoldingPolicy()
//   Options: (1) zero (default)
//                When in holding or captured state, an IPF
//                will be created with desired hdg of zero
//            (2) curr_hdg (probably should be default)
//                When in holding or captured state, an IPF
//                will be created with desired hdg matching
//                the vehicle's current heading
//            (3) setpt_hdg
//                When in captured state, an IPF will be
//                created with desired hdg matching the
//                vehicle's current hdg. When in holding
//                state, an IPF will be created with desired
//                hdg set to the heading toward the set pt.
//            (4) off
//                When in the capture or holding state, no
//                IPF will be created (behavior inactive)

bool BHV_ConvoyV21::handleConfigHoldingPolicy(string policy)
{
  if((policy != "zero") && (policy != "curr_hdg") &&
     (policy != "off")  && (policy != "setpt_hdg"))
    return(false);
  
  m_holding_policy = policy;

  return(true);
}


//-----------------------------------------------------------
// Procedure: checkParamCollective()
//      Note: Invoked after all params have been set, either upon
//            mission startup, or after a full string up dynamic
//            updates.
//            Will check relationships between parameter settings.
//   Returns: Empty string if all is ok.
//            Error message otherwise.

string BHV_ConvoyV21::checkParamCollective()
{
  string status = m_spd_policy_base.status();
  if(status != "ok")
    return("base spd_policy: " + status);

  if(m_slip_radius < m_capture_radius)
    return("slip_radius smaller than capture_radius");

  if(m_marker_tail.getInterMarkRange() >= m_marker_tail.getMaxTailLength())
    return("tail_length_max should be greater than inter_mark_range");
    
  return("");
}


//-----------------------------------------------------------
// Procedure: onSetParamComplete()
//      Note: If compression is non-zero, it is applied here
//            to the base speed policy.

void BHV_ConvoyV21::onSetParamComplete()
{
  postEventMessage("onSetParamComplete");
  // Prevailing speed policy is initially a copy of base policy
  m_spd_policy = m_spd_policy_base;
  // Then the current compression is applied
  m_spd_policy.compress(m_compression);
  m_spd_policy.setVName(tolower(m_us_name));
}


//-----------------------------------------------------------
// Procedure: onParamUpdate()

void BHV_ConvoyV21::onParamUpdate(string param)
{
  if(param == "contact")
    clearMarkerTail();
  //cout << "onParamUpdate: Marker Tail Cleared." << endl;
}



//-----------------------------------------------------------
// Procedure: onRunState

IvPFunction *BHV_ConvoyV21::onRunState() 
{
  // =====================================================
  // Part 1: Sanity checks
  // =====================================================
  if(!updatePlatformInfo())
    return(0);

  string spd_policy_status = m_spd_policy.status();
  if(spd_policy_status != "ok") {
    postWMessage("spd_policy error: " + spd_policy_status);
    return(0);
  }  
    
  // =====================================================
  // Part 2: Marker Maintenance
  // =====================================================
  // Part 2A: handle possible drop of aft marker
  bool new_aft_marker = false;
  //cout << "Stage 1: marker_tail.size:" << m_marker_tail.size();
  bool dropped_marker = checkDropAftMarker();
  //cout << "Stage 2: marker_tail.size:" << m_marker_tail.size();
  if(dropped_marker) {
    postFlags(m_marker_flags);
    new_aft_marker = true;
  }

  // If marker_tail is empty, the soon-to-be newly dropped marker
  // will be a *new* aft marker.
  if(m_marker_tail.empty())
    new_aft_marker = true;
  
  // Part 2B: Handle updated contact position, markers
  bool marker_added = m_marker_tail.handleNewContactPos(m_cnx, m_cny, m_cnh);
  //cout << "Stage 3: marker_tail.size:" << m_marker_tail.size();
  if(marker_added) {
    ConvoyMarker marker = m_marker_tail.getLeadMarker();
    drawMarker(marker);
  }

  //cout << "New Aft Marker:" << boolToString(new_aft_marker) << endl;
  if(new_aft_marker) {
    //cout << "New Aft Marker" << endl;
    ConvoyMarker aft_marker = m_marker_tail.getAftMarker();    
    drawMarker(aft_marker, "white");
  }
  //else
  //  cout << "NOT New Aft Marker" << endl;

  
  // Part 2C: Sanity check. Should never have an empty marker tail,
  // unless inter marker gap bigger than max tail length.
  if(m_marker_tail.empty()) {
    //cout << "2C: Empty Marker Tail" << endl;
    postWMessage("Empty Marker Tail");
    return(0);
  }

  // ======================================================
  // Part 3: Update the waypoint to be driven toward
  // ======================================================
  setCurrentMarker();
  
  // ======================================================
  // Part 4: Update the contact speed and our own goal speed
  // ======================================================
  handleNewContactSpd(m_cnv);
  m_set_speed = m_spd_policy.getSpdFromPolicy(m_cnv_avg_2sec,
					      m_contact_range,
					      m_convoy_range);

  // If additional max_speed constraint is enabled, apply here
  // Usually this is just for mission testing
  if((m_max_speed > 0) && (m_set_speed > m_max_speed))
    m_set_speed = m_max_speed;
  
  // ======================================================
  // Part 5: Update the metrics, post convoy_flags
  // ======================================================
  updateMetrics();
  postFlags(m_convoy_flags);
  postRecap(dropped_marker);
  postSpdPolicy();
  
  // Generate the IvP function 
  IvPFunction *ipf = buildOF();
  return(ipf);
}


//-----------------------------------------------------------
// Procedure: setCurrentMarker()

void BHV_ConvoyV21::setCurrentMarker()
{
  cout << "setCurrentMarker: tail_size=" << m_marker_tail.size() << endl;
  if(m_marker_tail.size() != 0) {
    ConvoyMarker marker = m_marker_tail.getAftMarker();
    m_wptx = marker.getX();
    m_wpty = marker.getY();
  }
  else {
    m_wptx = m_cnx;
    m_wpty = m_cny;
  }

  cout << "wpt: x=" << m_wptx << ", y=" << m_wpty << endl;
}

//-----------------------------------------------------------
// Procedure: checkDropAftMarker()

bool BHV_ConvoyV21::checkDropAftMarker()
{
  if(m_marker_tail.empty())
    return(false);

  ConvoyMarker aft_marker = m_marker_tail.getAftMarker();

  double dist = m_marker_tail.distToAftMarker(m_osx, m_osy);

  bool marker_dropped = false;
  
  // Case 1: Simplest check is the capture radius
  if(dist < m_capture_radius) {
    m_marker_tail.dropAftMarker();
    marker_dropped = true;
  }

  // Case 2: If inside the slip radius, check if ownship crossed line
  // perpendicular to line from aft marker to near-aft marker.
  if(!marker_dropped && (dist < m_slip_radius)) {
    // Calculate the next point on the marker list. 
    //
    //               wpt             nextpt         | 
    //                 o----------------o           |
    //                  \ angle                     | 
    //                   \                          |  
    //                    \                         |
    //                     o--->                    | 
    //                  ownship                     |                   

    // If only one marker, next "marker" will be contact position
    double next_x = m_cnx;
    double next_y = m_cny;

    if(m_marker_tail.size() > 1) {
      ConvoyMarker marker = m_marker_tail.getNearAftMarker();
      double next_x = marker.getX();
      double next_y = marker.getY();
    }
    double angle = angleFromThreePoints(m_wptx, m_wpty, m_osx, m_osy,
					next_x, next_y);
    if(angle < 90) {
      m_marker_tail.dropAftMarker();
      marker_dropped = true;
    }
  }

  // Case 3: If aft marker was not captured, check if the aft marker
  // should be dropped based on exceeding the max tail length.
  if(!marker_dropped) {
    string msg;
    marker_dropped = m_marker_tail.checkDropAftMarker(msg);
  }

  // Last step: If marker was dropped, count it and erase it.
  if(marker_dropped) {
    m_dropped_markers++;
    eraseMarker(aft_marker);
  }
    
  return(marker_dropped);
}


//-----------------------------------------------------------
// Procedure: handleNewContactSpd()

void BHV_ConvoyV21::handleNewContactSpd(double cnv)
{
  double curr_time = getBufferCurrTime();
  m_cn_spd_value.push_front(cnv);
  m_cn_spd_tstamp.push_front(curr_time);

  // Part 1A: Ensure behavior spd queue is no older than 5 secs.
  bool done = false;
  while(!done && !m_cn_spd_tstamp.empty()) {
    if((curr_time - m_cn_spd_tstamp.back()) > 5) {
      m_cn_spd_value.pop_back();
      m_cn_spd_tstamp.pop_back();
    }
    else
      done = true;
  }

  // Part 1B: Get the 5 second average
  double total_spd = 0;
  unsigned int cnt = 0;
  list<double>::iterator p;
  for(p=m_cn_spd_value.begin(); p!=m_cn_spd_value.end(); p++) {
    double this_spd = *p;
    total_spd += this_spd;
    cnt++;
  }
  if(cnt > 0)
    m_cnv_avg_5sec = total_spd / (double)(cnt);

  // Part 2A: Build a copy of the spd queue only over 2 secs
  list<double> cn_spd_value  = m_cn_spd_value;
  list<double> cn_spd_tstamp = m_cn_spd_tstamp;
  done = false;
  while(!done && !cn_spd_tstamp.empty()) {
    if((curr_time - cn_spd_tstamp.back()) > 2) {
      cn_spd_value.pop_back();
      cn_spd_tstamp.pop_back();
    }
    else
      done = true;
  }

  // Part 2B: Get the 2 second average
  total_spd = 0;
  cnt = 0;
  for(p=cn_spd_value.begin(); p!=cn_spd_value.end(); p++) {
    double this_spd = *p;
    total_spd += this_spd;
    cnt++;
  }
  if(cnt > 0)
    m_cnv_avg_2sec = total_spd / (double)(cnt);
}

//-----------------------------------------------------------
// Procedure: updateMetrics()

void BHV_ConvoyV21::updateMetrics() 
{
  // Part 1: Update direct raw metrics
  m_tail_range = m_marker_tail.distToAftMarker(m_osx, m_osy);
  m_tail_angle = m_marker_tail.tailAngle(m_osx, m_osy);
  m_marker_bng = m_marker_tail.markerBearing(m_osx, m_osy, m_osh);
  m_alignment = m_tail_angle + m_marker_bng;

  if(m_marker_tail.size() == 0) 
    m_convoy_range = m_contact_range;
  else
    m_convoy_range = m_tail_range + m_marker_tail.getMarkerTailLen();

  m_range_delta = m_convoy_range - m_spd_policy.getIdealConvoyRng();
  if(m_range_delta < 0)
    m_range_delta *= -1;

  m_track_error = m_marker_tail.getTrackError(m_osx, m_osy);
}

//-----------------------------------------------------------
// Procedure: buildOF()

IvPFunction *BHV_ConvoyV21::buildOF() 
{
  IvPFunction *ipf = 0;

  // ======================================================
  // Part 0: Determine if we are holding and waiting for marker tail
  // to evolve before moving.
  // ======================================================
  bool holding = false;
  if(m_aft_patience && !m_marker_tail.aftMarkerClosest(m_osx, m_osy))
    holding = true;
  
  // ======================================================
  // Part 1: Build the Speed ZAIC
  // ======================================================
  if(holding)
    m_set_speed = 0;

  ZAIC_SPD spd_zaic(m_domain, "speed");
  spd_zaic.setMedSpeed(m_set_speed);

  string mode = m_spd_policy.getCorrectionMode();
  if(mode == "close")
    spd_zaic.setMinSpdUtil(50);
  else if(mode == "ideal_close")
    spd_zaic.setMinSpdUtil(25);
  else if(mode == "ideal_far")
    spd_zaic.setMaxSpdUtil(25);
  else if(mode == "far")
    spd_zaic.setMaxSpdUtil(50);
  else if(mode == "full_lag")
    spd_zaic.setMaxSpdUtil(75);
  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  if(!spd_ipf) {
    postWMessage("Failure on the SPD ZAIC via ZAIC_SPD utility");
    return(0);
  }
  
  // ======================================================
  // Part 2: Build the Course ZAIC
  // ======================================================
  double set_hdg = relAng(m_osx, m_osy, m_wptx, m_wpty);
  if(holding) {
    if(m_holding_policy == "zero")
      set_hdg = 0;
    else if(m_holding_policy == "curr_hdg")
      set_hdg = m_osh;
    else if(m_holding_policy == "off")
      return(0);
    // If holding_policy is setpt_hdg, this is as set_hdg
  }
  
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setValueWrap(true);
  crs_zaic.setParams(set_hdg, 0, 180, 50, 0, 100);
  int ix = crs_zaic.addComponent();
  crs_zaic.setParams(m_osh, 30, 180, 5, 0, 20, ix);
  
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);  
  if(!crs_ipf) 
    postWMessage("Failure on the CRS ZAIC");

  // ======================================================
  // Part 3: Couple the two 1D functions into a 2D function
  // ======================================================
  OF_Coupler coupler;
  ipf = coupler.couple(crs_ipf, spd_ipf, m_patience, (100-m_patience));
  if(!ipf)
    postWMessage("Failure on the CRS_SPD COUPLER");

  return(ipf);
}


//-----------------------------------------------------------
// Procedure: onRunToIdleState()

void BHV_ConvoyV21::onRunToIdleState()
{
  postIdleRecap();
  clearMarkerTail();
}

//-----------------------------------------------------------
// Procedure: clearMarkerTail()

void BHV_ConvoyV21::clearMarkerTail()
{
  // Part 1: Visuals: Get all markers so each can be erased.
  list<ConvoyMarker> markers = m_marker_tail.getMarkers();

  list<ConvoyMarker>::iterator p;
  for(p=markers.begin(); p!=markers.end(); p++) {
    ConvoyMarker marker = *p;
    eraseMarker(marker);
  }

  // Part 2: Clear the markers from from the marker_tail 
  m_marker_tail.clear();
}

//-----------------------------------------------------------
// Procedure: drawMarker()

void BHV_ConvoyV21::drawMarker(ConvoyMarker marker, string color)
{
  if(!marker.valid())
    return;
  
  double mx = marker.getX();
  double my = marker.getY();
  unsigned int id = marker.getID();

  XYPoint point(mx, my);
  string label = tolower(m_us_name) + "_";
  label += tolower(m_contact) + "_" + uintToString(id);
  
  point.set_vertex_size(m_hint_marker_size);
  point.set_label(label);
  point.set_color("label", m_hint_marker_label_color);

  if(isColor(color))
    point.set_color("vertex", color);
  else
    point.set_color("vertex", m_hint_marker_color);
  
  string spec = point.get_spec();
  postMessage("VIEW_POINT", spec);
}

//-----------------------------------------------------------
// Procedure: eraseMarker()

void BHV_ConvoyV21::eraseMarker(ConvoyMarker marker)
{
  if(!marker.valid())
    return;

  double mx = marker.getX();
  double my = marker.getY();
  unsigned int id = marker.getID();

  XYPoint point(mx, my);
  string label = tolower(m_us_name) + "_";
  label += tolower(m_contact) + "_" + uintToString(id);
  
  point.set_label(label);
  point.set_active(false);
  
  string spec = point.get_spec();
  postMessage("VIEW_POINT", spec);
}


//-----------------------------------------------------------
// Procedure: handleParamVisualHints()

bool BHV_ConvoyV21::handleParamVisualHints(string hints)
{
  vector<string> svector = parseStringQ(hints, ',');

  for(unsigned int i=0; i<svector.size(); i++) {
    string hint  = svector[i];
    string param = tolower(biteStringX(hint, '='));
    string value = hint;
    
    if(param == "marker_color")
      return(setColorOnString(m_hint_marker_color, value));
    else if(param == "marker_label_color")
      return(setColorOnString(m_hint_marker_label_color, value));
    else if(param == "marker_size")
      return(setNonNegDoubleOnString(m_hint_marker_size, value));
    else
      return(false);
  }
  return(true);
}

//-----------------------------------------------------------
// Procedure: handleParamCompression()

bool BHV_ConvoyV21::handleParamCompression(string sval)
{
  double dval = atof(sval.c_str());
  if((dval < 0) || (dval > 1))
    return(false);

  m_compression = dval;

  return(true);    
}

//-----------------------------------------------------------
// Procedure: postRecap()
//      Note: Dropped Marker info is passed in to allow non-verbose
//            mode to hold back recaps to only occure when a marker
//            is dropped.
//      Note: The static recap will only be posted by the helm when
//            it changes. Helm duplication filter will handle this.
//      Note: Both publications can be silenced with the generic
//            post_mapping = CONVOY_RECAP,silent parameter, defined
//            at the IvPBehavior superclass level.

void BHV_ConvoyV21::postRecap(bool dropped_marker)
{
  //========================================================
  // Part 1: Build recap of items not likely to change often
  //========================================================
  ConvoyStatRecap stat_recap;
  stat_recap.setLeader(tolower(m_contact));
  stat_recap.setFollower(tolower(m_us_name));
  stat_recap.setIdealRng(m_spd_policy.getIdealConvoyRng());
  stat_recap.setCompression(m_compression);
  stat_recap.setIndex(m_stat_recap_index);
			 
  postMessage("CONVOY_STAT_RECAP", stat_recap.getSpec());

  //========================================================
  // Part 2: Build recap of items likely to change often
  //========================================================
  if(!m_post_recap_verbose)
    return;

  ConvoyRecap recap;
  recap.setVName(tolower(m_us_name));
  recap.setCName(tolower(m_contact));
  recap.setConvoyRng(m_convoy_range);
  recap.setConvoyRngDelta(m_range_delta);
  recap.setTailRng(m_tail_range);
  recap.setTailAng(m_tail_angle);
  recap.setMarkerBng(m_marker_bng);
  recap.setTrackErr(m_track_error);
  recap.setAlignment(m_alignment);
  recap.setSetSpd(m_set_speed);
  recap.setAvg2(m_cnv_avg_2sec);
  recap.setAvg5(m_cnv_avg_5sec);
  recap.setCorrMode(m_spd_policy.getCorrectionMode());
  recap.setTailCnt(m_marker_tail.size());
  recap.setIndex(m_recap_index);
  recap.setTimeUTC(getBufferCurrTime());
  
  if(!m_marker_tail.empty()) {
    ConvoyMarker marker = m_marker_tail.getAftMarker();
    recap.setMarkerX(marker.getX());
    recap.setMarkerY(marker.getY());
    recap.setMarkerID(marker.getID());
  }

  m_recap_index++;
  postMessage("CONVOY_RECAP", recap.getSpec());
}

//-----------------------------------------------------------
// Procedure: postIdleRecap()

void BHV_ConvoyV21::postIdleRecap()
{
  ConvoyStatRecap stat_recap;
  stat_recap.setLeader(tolower(m_contact));
  stat_recap.setFollower(tolower(m_us_name));
  stat_recap.setIdle(true);
  postMessage("CONVOY_STAT_RECAP", stat_recap.getSpec());

  if(!m_post_recap_verbose)
    return;

  ConvoyRecap recap;
  recap.setVName(tolower(m_us_name));
  recap.setCName(tolower(m_contact));
  recap.setIdle(true);
  postMessage("CONVOY_RECAP", recap.getSpec());
}

//-----------------------------------------------------------
// Procedure: postSpdPolicy()

void BHV_ConvoyV21::postSpdPolicy()
{
  string str_spd_policy = m_spd_policy.getSpec();
  postMessage("CONVOY_SPD_POLICY", str_spd_policy);
}


//-----------------------------------------------------------
// Procedure: expandMacros()

string BHV_ConvoyV21::expandMacros(string sdata)
{
  // =======================================================
  // First expand the macros defined at the superclass level
  // =======================================================
  sdata = IvPContactBehavior::expandMacros(sdata);

  double ideal_convoy_rng = m_spd_policy.getIdealConvoyRng();
  string correction_mode = m_spd_policy.getCorrectionMode();

  sdata = macroExpand(sdata, "LEADER", tolower(m_contact));
  sdata = macroExpand(sdata, "CONVOY_RNG", m_convoy_range);
  sdata = macroExpand(sdata, "RNG_DELTA", m_range_delta);
  sdata = macroExpand(sdata, "IDEAL_RNG", ideal_convoy_rng);
  sdata = macroExpand(sdata, "COMP", m_compression);

  sdata = macroExpand(sdata, "TAIL_RNG", m_tail_range);
  sdata = macroExpand(sdata, "TAIL_ANG", m_tail_angle);
  sdata = macroExpand(sdata, "TRK_ERR", m_track_error);
  sdata = macroExpand(sdata, "MARKER_BNG", m_marker_bng);
  sdata = macroExpand(sdata, "ALIGNMENT", m_alignment);

  sdata = macroExpand(sdata, "SET_SPD", m_set_speed);
  sdata = macroExpand(sdata, "CMODE", correction_mode);
  sdata = macroExpand(sdata, "AVG_SPD2", m_cnv_avg_2sec);
  sdata = macroExpand(sdata, "AVG_SPD5", m_cnv_avg_5sec);

  // marker count / tail size
  // num dropped, num reached
  
  return(sdata);
}


