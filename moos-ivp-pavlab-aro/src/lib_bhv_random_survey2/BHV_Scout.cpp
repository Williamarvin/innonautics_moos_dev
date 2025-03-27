/*****************************************************************/
/*    NAME: T. Paine adapted from 2.680 work by M.Benjamin,      */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_Scout.cpp                                        */
/*    DATE: April 30th 2022                                      */
/*****************************************************************/

#include <cstdlib>
#include <math.h>
#include "BHV_Scout.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "BuildUtils.h"
#include "GeomUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "XYFormatUtilsPoly.h"

#include <ctime>         // For use of the time function
#include <unistd.h>      // For use of the getpid function

using namespace std;

//-----------------------------------------------------------
// Constructor()

BHV_Scout::BHV_Scout(IvPDomain gdomain) : 
  IvPBehavior(gdomain)
{
  IvPBehavior::setParam("name", "random_survey2");
 
  // Default values for behavior state variables
  m_osx  = 0;
  m_osy  = 0;

  // All distances are in meters, all speed in meters per second
  // Default values for configuration parameters 
  m_desired_speed  = 3; 
  m_capture_radius = 10;

  m_pt_set = false;
  m_poly_posted = false;
  
  addInfoVars("NAV_X, NAV_Y");
  addInfoVars("SCOUTED_SWIMMER");
  
  m_region_update_var = "RESCUE_REGION";
  m_edge_color = "white";
  m_vertex_color = "blue";
  m_point_color = "orange";
  m_label_prefix = ""; 

  
}

//---------------------------------------------------------------
// Procedure: setParam() - handle behavior configuration parameters

bool BHV_Scout::setParam(string param, string val) 
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);
  
  bool handled = true;
  if(param == "capture_radius")
    handled = setPosDoubleOnString(m_capture_radius, val);
  else if(param == "desired_speed")
    handled = setPosDoubleOnString(m_desired_speed, val);
  else if(param == "tmate")
    handled = setNonWhiteVarOnString(m_tmate, val);
  else if(param == "region_update_var")
    handled = setNonWhiteVarOnString(m_region_update_var, val);
  else if(param == "label_prefix")
    handled = setNonWhiteVarOnString(m_label_prefix, val);
  else if(param == "edge_color"){
    handled = setNonWhiteVarOnString(m_edge_color, val);
    handled = handled && isColor(m_edge_color);
  }else if(param == "vertex_color"){
    handled = setNonWhiteVarOnString(m_vertex_color, val);
    handled = handled && isColor(m_vertex_color);
  }else if(param == "point_color"){
    handled = setNonWhiteVarOnString(m_point_color, val);
    handled = handled && isColor(m_point_color); 
  }else
    handled = false;

  addInfoVars(m_region_update_var); 

  srand(time(NULL));
  
  return(handled);
}

//-----------------------------------------------------------
// Procedure: onRunToIdleState()
//      Note: Invoked automatically by the helm when the behavior

void BHV_Scout::onRunToIdleState() 
{

}

//-----------------------------------------------------------
// Procedure: onEveryState()

void BHV_Scout::onEveryState(string str) 
{
  if(getBufferVarUpdated(m_region_update_var))
    m_pt_set = false;
  
  if(!getBufferVarUpdated("SCOUTED_SWIMMER"))
    return;

  string report = getBufferStringVal("SCOUTED_SWIMMER");
  if(report == "")
    return;

  if(m_tmate == "") {
    postWMessage("Mandatory Teammate name is null");
    return;
  }
  postOffboardMessage(m_tmate, "SWIMMER_ALERT", report);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_Scout::onIdleState() 
{
  m_curr_time = getBufferCurrTime();

  // Clean up if needed
  if (m_pt_set){
    postViewPoint(false);
    m_pt_set = false;
  }

  if (m_poly_posted)
    postViewPolygon(false);
      

  
  
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_Scout::onRunState() 
{
  // Part 1: Get vehicle position from InfoBuffer and post a 
  // warning if problem is encountered
  bool ok1, ok2;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  if(!ok1 || !ok2) {
    postWMessage("No ownship X/Y info in info_buffer.");
    return(0);
  }
  
  // Part 2: Determine if the vehicle has reached the destination 
  // point and if so, declare completion.
  updateScoutPoint();
  double dist = hypot((m_ptx-m_osx), (m_pty-m_osy));
  //postEventMessage("Dist=" + doubleToStringX(dist,1));
  if(dist <= m_capture_radius) {
    m_pt_set = false;
    postViewPoint(false);
    return(0);
  }

  // Part 3: Post the waypoint as a string for consumption by 
  // a viewer application.
  postViewPoint(true);

  // Part 4: Build the IvP function 
  IvPFunction *ipf = buildFunction();
  if(ipf == 0) 
    postWMessage("Problem Creating the IvP Function");
  
  return(ipf);
}

//-----------------------------------------------------------
// Procedure: updateScoutPoint()

void BHV_Scout::updateScoutPoint()
{
  if(m_pt_set)
    return;

  string region_str = getBufferStringVal(m_region_update_var);
  if(region_str == "")
    postWMessage("Unknown region in " + m_region_update_var);
  else
   postRetractWMessage("Unknown region in " + m_region_update_var);

  XYPolygon region = string2Poly(region_str);
  //postWMessage("converted region, spec = " + region.get_spec());
  
  if(!region.is_convex()) {
    postWMessage("Badly formed REGION");
    return;
  }
  m_rescue_region = region;

  if (!m_poly_posted)
    postViewPolygon(true);

  
  double ptx = 0;
  double pty = 0;
  bool ok = false;

  if (m_rescue_region.contains(m_osx, m_osy)){
    unsigned long int tseed = time(NULL);
    unsigned long int pseed = getpid() + 1;
    unsigned int rseed = (tseed*pseed) % 50000;
    
    ok = randSeededPointInPoly(m_rescue_region, ptx, pty, rseed, 1000);
  } else{
    XYPolygon shrunk_rescue_region = m_rescue_region;
    shrunk_rescue_region.grow_by_amt(-1.0 * m_capture_radius * 3.0);
    ok = shrunk_rescue_region.closest_point_on_poly(m_osx, m_osy, ptx, pty);
    if (!ok)
      ok = m_rescue_region.closest_point_on_poly(m_osx, m_osy, ptx, pty); 
  }
  
  if(!ok) {
    postWMessage("Unable to generate scout point");
    return;
  }
    
  m_ptx = ptx;
  m_pty = pty;
  m_pt_set = true;
  string msg = "New pt: " + doubleToStringX(ptx) + "," + doubleToStringX(pty);
  postEventMessage(msg);
}

//-----------------------------------------------------------
// Procedure: postViewPoint()

void BHV_Scout::postViewPoint(bool viewable) 
{

  XYPoint pt(m_ptx, m_pty);
  pt.set_vertex_size(5);
  pt.set_vertex_color(m_point_color);
  pt.set_label(m_label_prefix + ":" + m_us_name + "'s next waypoint");
  
  string point_spec;
  if(viewable)
    point_spec = pt.get_spec("active=true");
  else
    point_spec = pt.get_spec("active=false");
  postMessage("VIEW_POINT", point_spec);

}

void BHV_Scout::postViewPolygon(bool viewable)
{
  // and also post the polygon here
  m_rescue_region.set_active(viewable); 
  m_rescue_region.set_color("edge", m_edge_color);
  m_rescue_region.set_color("vertex", m_vertex_color);
  m_rescue_region.set_label(m_label_prefix + ":" + m_us_name + "'s scout region");
  postMessage("VIEW_POLYGON", m_rescue_region.get_spec());

  m_poly_posted = viewable; 
  
}


//-----------------------------------------------------------
// Procedure: buildFunction()

IvPFunction *BHV_Scout::buildFunction() 
{
  if(!m_pt_set)
    return(0);
  
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  spd_zaic.setSummit(m_desired_speed);
  spd_zaic.setPeakWidth(0.5);
  spd_zaic.setBaseWidth(1.0);
  spd_zaic.setSummitDelta(0.8);  
  if(spd_zaic.stateOK() == false) {
    string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }
  
  double rel_ang_to_wpt = relAng(m_osx, m_osy, m_ptx, m_pty);
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(rel_ang_to_wpt);
  crs_zaic.setPeakWidth(0);
  crs_zaic.setBaseWidth(180.0);
  crs_zaic.setSummitDelta(0);  
  crs_zaic.setValueWrap(true);
  if(crs_zaic.stateOK() == false) {
    string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }

  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();

  OF_Coupler coupler;
  IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);

  return(ivp_function);
}
