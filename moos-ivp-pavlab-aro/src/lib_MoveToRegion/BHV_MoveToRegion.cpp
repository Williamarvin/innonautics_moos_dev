/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_MoveToRegion.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_MoveToRegion.h"


using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_MoveToRegion::BHV_MoveToRegion(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y");

  // Initialize state variables
  m_osx = 0;
  m_osy = 0;
  m_setpt_x = 0;
  m_setpt_y = 0;
  m_setpt_x_prev = 0;
  m_setpt_y_prev = 0; 
  m_cruise_speed = 0.0;
  m_patience     = 80;

  addInfoVars("NAV_X, NAV_Y");

  m_stale_nav_thresh = 2.0;
  m_region_count = 0; 

  
  m_setpt_viewable    = true;
  m_hint_setpt_size   = 4;
  m_hint_setpt_color  = "yellow";
  m_hint_setpt_lcolor = "off";
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_MoveToRegion::setParam(string param, string value)
{
  if(IvPBehavior::setParam(param, value))
    return(true);
  
  // Convert the parameter to lower case for more general matching
  param = tolower(param);
  
  bool handled = true; 
  if(param == "region"){
    handled = handleConfigRegion(value);
  } else if(param == "speed") {
    handled = setNonNegDoubleOnString(m_cruise_speed, value);
  } else if(param == "stale_nav_thresh") {
    handled = setNonNegDoubleOnString(m_stale_nav_thresh, value);
  } else if(param == "patience") {
    handled = setNonNegDoubleOnString(m_patience, value);
  }  else if(param == "setpt_viewable")
    handled = setBooleanOnString(m_setpt_viewable, value);
  else if(param == "visual_hints")  {
    vector<string> svector = parseStringQ(value, ',');
    for(unsigned int i=0; i<svector.size(); i++) 
      handled = handled && handleConfigVisualHint(svector[i]);
  } else {
    handled = false;
  }

  return(handled);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_MoveToRegion::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_MoveToRegion::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_MoveToRegion::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_MoveToRegion::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_MoveToRegion::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_MoveToRegion::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_MoveToRegion::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_MoveToRegion::onRunState()
{


  // Part 0: Update ownship and proxonoi information
  bool ok = updateOwnshipPosition();
  if(!ok)
    return(0);

  // Part 2: check for completion
  if(checkForCompletion())
    return(0);

  // Part 2: Update setpoint
  bool ok2 = updateSetPoint();
  if (!ok2)
    return(0);
  
  // Part 3: Build the IvP function
  IvPFunction *ipf = 0;

  // Part 3a) build the speed ZAIC
  double set_spd = m_cruise_speed;
  //  if((m_state != "activated") && (m_state != "transiting")) 
    set_spd = 0;

  ZAIC_SPD spd_zaic(m_domain, "speed");
  spd_zaic.setParams(m_cruise_speed, 0.1, m_cruise_speed+0.4, 85, 20);  

  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  if(!spd_ipf)
    postWMessage("Failure on the SPD ZAIC via ZAIC_PEAK utility");

  // Part 3b) build the heading ZAIC
  double angle_to_setpt = relAng(m_osx, m_osy, m_setpt_x, m_setpt_y);

  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(angle_to_setpt);
  crs_zaic.setBaseWidth(180);
  crs_zaic.setValueWrap(true);

  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);  
  if(!crs_ipf) 
    postWMessage("Failure on the CRS ZAIC");

  // Couple the two together
  OF_Coupler coupler;
  ipf = coupler.couple(crs_ipf, spd_ipf, m_patience, 100-m_patience);
  if(!ipf)
    postWMessage("Failure on the CRS_SPD COUPLER");

  ipf->getPDMap()->normalize(0,100);
  
  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}



bool BHV_MoveToRegion::updateOwnshipPosition() 
{
  //========================================================= 
  // Part 1: Update ownship position and check for errors
  //========================================================= 
  bool ok_x = true;
  bool ok_y = true;
  double new_osx = getBufferDoubleVal("NAV_X", ok_x);
  double new_osy = getBufferDoubleVal("NAV_Y", ok_y);

  if(!ok_y || !ok_y) {
    postEMessage("ownship NAV_X/Y  info not found.");
    return(false);
  }
  m_osx = new_osx;
  m_osy = new_osy;
  
  //========================================================= 
  // Part 2: Check for staleness of ownship NAV information
  //========================================================= 
  double tstamp_osx = getBufferTimeVal("NAV_X");
  double tstamp_osy = getBufferTimeVal("NAV_Y");
  if((tstamp_osx > m_stale_nav_thresh) ||
     (tstamp_osy > m_stale_nav_thresh)) {
    postEMessage("ownship NAV_X/Y info is stale.");
    return(false);
  }

  return(true);
}


bool BHV_MoveToRegion::handleConfigRegion(string str)
{
  
  // Part 1: Parse the polygon string and check for validity
  XYPolygon poly = string2Poly(str);
  if(!poly.is_convex())
    return(false);

  // Part 2: Check region has a label, if not, make unique label
  string label = poly.get_label();
  if(label == "") 
    label = "reg_" + uintToString(m_region_count);
  poly.set_label(label);

  m_poly_region = poly;
  m_region_count +=1; 
  
  return(true);
}


//-----------------------------------------------------------
// Procedure: updateSetPoint()
//   Returns: true if (a) region is convex
  
bool BHV_MoveToRegion::updateSetPoint()
{
  // sanity check
  if(!m_poly_region.is_convex())
    return(false);

  double new_setpt_x = 0.0;
  double new_setpt_y = 0.0;
  bool ok = m_poly_region.closest_point_on_poly(m_osx, m_osy, new_setpt_x, new_setpt_y);
  if (!ok)
    return(false);

  // Part 2: If setpt has moved, update the postings
  if((m_setpt_x != new_setpt_x) || (m_setpt_y != new_setpt_y)) {
    m_setpt_x = new_setpt_x;
    m_setpt_y = new_setpt_y;
    postSetPoint();
  }
  return(true); 
}

//-----------------------------------------------------------
// Procedure: postSetPoint()
  
void BHV_MoveToRegion::postSetPoint()
{
  if(m_hint_setpt_size == 0)
    return;
  if(m_hint_setpt_color == "invisible")
    return;
  if(!m_setpt_viewable)
    return;

  if(commsPolicy() == "dire")
    return;

  if(commsPolicy() == "lean") {
    double dist_to_prev = distPointToPoint(m_setpt_x_prev, m_setpt_y_prev, m_setpt_x, m_setpt_y); 
    if(dist_to_prev < 1)
      return;
  }

  m_setpt_x_prev = m_setpt_x;
  m_setpt_y_prev = m_setpt_y;
  
  XYPoint point(m_setpt_x, m_setpt_y);
  point.set_label(m_us_name + "setpt");
  point.set_label_color(m_hint_setpt_lcolor);
  point.set_vertex_size(m_hint_setpt_size);
  point.set_vertex_color(m_hint_setpt_color);
  point.set_duration(3);
  
  string spec = point.get_spec();
  postMessage("VIEW_POINT", spec);
}

//-----------------------------------------------------------
// Procedure: handleConfigVisualHint()

bool BHV_MoveToRegion::handleConfigVisualHint(string hint)
{
  string param = tolower(biteStringX(hint, '='));
  string value = hint;
  double dval  = atof(value.c_str());
  
  if((param == "setpt_size") && isNumber(value) && (dval >= 0))
    m_hint_setpt_size = dval;
  else if((param == "setpt_color") && isColor(value))
    m_hint_setpt_color = value;
  else if((param == "setpt_label_color") && isColor(value))
    m_hint_setpt_lcolor = value;
  else
    return(false);

  return(true);  
}


//-----------------------------------------------------------
// Procedure: checkForCompletion()
bool BHV_MoveToRegion::checkForCompletion()
{
  if (!m_poly_region.contains(m_osx, m_osy)) {
    return(false); 
  }    
  
  setComplete();
  return(true);
}
