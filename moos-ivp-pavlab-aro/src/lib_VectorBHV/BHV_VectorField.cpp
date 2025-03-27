/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_VectorField.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_VectorField.h"
#include "ZAIC_PEAK.h"
#include "GeomUtils.h"
#include "AngleUtils.h"
#include <cmath>
#include "XYFormatUtilsPoly.h"
#include "OF_Coupler.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_VectorField::BHV_VectorField(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "vectorfield");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Initialize state variables
  m_osx = 0;
  m_osy = 0;
  m_ownship_in_region = false;
  m_region_center_x=0;
  m_region_center_y=0;
  m_cruise_speed = 0;
  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_VectorField::setParam(string param, string param_val)
{
  if(IvPBehavior::setParam(param, param_val))
    return(true);

  bool handled = false;
  if(param == "speed"){
    handled = setNonNegDoubleOnString(m_cruise_speed, param_val);
  }
  else if(param == "op_region"){
    handled = handleConfigOpRegion(param_val);
  }
  else if(param == "spin_rad"){
    handled = setNonNegDoubleOnString(m_spin_rad, param_val);
  }
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // // Get the numerical value of the param argument for convenience once
  // double double_val = atof(val.c_str());
  
  // if((param == "foo") && isNumber(val)) {
  //   // Set local member variables here
  //   return(true);
  // }
  // else if (param == "bar") {
  //   // return(setBooleanOnString(m_my_bool, val));
  // }

  // If not handled above, then just return false;
  return(handled);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_VectorField::onSetParamComplete()
{
}
//-----------------------------------------------------------
// Procedure: updateOwnshipPosition()
//   Returns: true if Nav info is found and not stale
//            false otherwise

bool BHV_VectorField::updateOwnshipPosition() 
{
  //========================================================= 
  // Part 1: Update ownship position and check for errors
  //========================================================= 
  bool ok_x = true;
  bool ok_y = true;
  double new_osx = getBufferDoubleVal("NAV_X", ok_x);
  double new_osy = getBufferDoubleVal("NAV_Y", ok_y);

  if(!ok_y || !ok_y) {
    postEMessage("ownship NAV_X/Y info not found.");
    return(false);
  }

  //========================================================= 
  // Part 3: Update ownship position and the odometer
  //========================================================= 
  m_osx = new_osx;
  m_osy = new_osy;
  m_ownship_in_region = m_op_region.contains(m_osx, m_osy);
  if(m_ownship_in_region){
    m_in=1;
  }
  else{
    m_in=0;
  }

  return(true);
}

//-----------------------------------------------------------
// Procedure: handleConfigOpRegion()
//   Returns: true OpRegion Poly is convex
  
bool BHV_VectorField::handleConfigOpRegion(string polystr)
{
  XYPolygon new_poly = string2Poly(polystr);
  if(!new_poly.is_convex()){
    return(false);
  }
  m_op_region = new_poly;
  m_region_center_x=m_op_region.get_center_x();
  m_region_center_y=m_op_region.get_center_y();
  m_region_radius=m_op_region.max_radius();
  postMessage("THECENTX",m_region_center_x);
  postMessage("THECENTY",m_region_center_y);
    double max_dist_so_far = 0;
  double cx = m_op_region.get_center_x();
  double cy = m_op_region.get_center_y();

  for(unsigned int i=0; i<m_op_region.size(); i++) {
    double delta_x = cx - m_op_region.get_vx(i);
    double delta_y = cy - m_op_region.get_vy(i);
    double dist = hypot(delta_x, delta_y);
    if(dist > max_dist_so_far)
      max_dist_so_far = dist;
      m_max_x=m_op_region.get_vx(i);
      m_max_y=m_op_region.get_vy(i);
  }
  m_region_radius=max_dist_so_far;
  postMessage("THERAD",m_region_radius);
  return(true);
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_VectorField::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_VectorField::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_VectorField::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_VectorField::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_VectorField::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_VectorField::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_VectorField::onRunState()
{
  cout << "onRunState()" << endl;
  
  // Part 1: Update ownship and proxonoi information
  bool ok = updateOwnshipPosition();
  if(!ok) {
    postMessage("VECTOR_DEBUG", "Unable to update ownship position");
    return(0);
  }









  // Part 1: Build the IvP function
    IvPFunction *ipf = buildOF();


  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  double dy=((m_osy-m_region_center_y)*(m_osy-m_region_center_y));
  double dx=((m_osx-m_region_center_x)*(m_osx-m_region_center_x));
  double dist_to_cent=sqrt(dx+dy);
  if(dist_to_cent>m_spin_rad){
    m_short=1;
  }
  else{
    m_short=1;
  }
  if(ipf)
    ipf->setPWT(m_priority_wt*m_in*m_short);

  return(ipf);
}
IvPFunction *BHV_VectorField::buildOF() 
{
  IvPFunction *ipf = 0;
  //===================================================
  // Part 1: Build the Speed ZAIC
  //===================================================
  IvPFunction *spd_ipf = 0;  
  ZAIC_PEAK spd_zaic(m_domain, "speed");
    double mag1=((m_osy-m_region_center_y));//*sqrt(abs(m_osy-m_region_center_y)));
    double mag2=((m_osx-m_region_center_x));//*sqrt(abs(m_osx-m_region_center_x)));
    double mag=sqrt((mag1*mag1)+(mag2*mag2));
    double mag1n=((m_max_y-m_region_center_y));//*sqrt(abs(m_max_y-m_region_center_y)));
    double mag2n=((m_max_x-m_region_center_x));//*sqrt(abs(m_max_x-m_region_center_x)));
    double max_region=sqrt((mag1n*mag1n)+(mag2n*mag2n));
   /////////////////////////////////
   //DONT FORGET 100/////////////
   ////////////////////////
    double rel=mag/100;
    postMessage("REGION_RAD",m_region_radius);
    postMessage("DISTC",mag);

    
    double speedord=m_cruise_speed;
    if (max_region > 0.0)
      speedord = (mag/max_region)*m_cruise_speed;
    
    //double speedord=m_cruise_speed;
    postMessage("ORDSPEED",speedord);
    double peak_width = speedord / (1.4);
    spd_zaic.setParams(speedord, peak_width, 1.6, 20, 0, 100);
  spd_ipf = spd_zaic.extractIvPFunction();
  if(!spd_ipf)
    postWMessage("Failure on the SPD ZAIC via ZAIC_PEAK utility");
  
  //===================================================
  // Part 2: Build the Course ZAIC
  //===================================================
 
  ZAIC_PEAK crs_zaic(m_domain, "course");

    double y=m_osy-m_region_center_y;
    double x=-1*(m_osx-m_region_center_x);
    double resultt = atan2 (y,x) * 180 / M_PI;
    if(resultt<0)
    resultt = (resultt + 360);
    postMessage("RELANG",resultt+90);
    postMessage("CENTX",x);
    postMessage("CENTY",y);
    crs_zaic.setSummit(resultt+10);
    crs_zaic.setBaseWidth(90);
    crs_zaic.setValueWrap(true);
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);  
  if(!crs_ipf) 
    postWMessage("Failure on the CRS ZAIC");
  
  OF_Coupler coupler;
  ipf = coupler.couple(crs_ipf, spd_ipf, 0.5, 0.5);
  if(!ipf)
    postWMessage("Failure on the CRS_SPD COUPLER");

  return(ipf);
}
