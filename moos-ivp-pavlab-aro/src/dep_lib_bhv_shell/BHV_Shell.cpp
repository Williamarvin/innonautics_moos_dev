/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Shell.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_Shell.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_Shell::BHV_Shell(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y");

  m_input_heading_var = "";
    m_input_speed_var = "";
       m_stale_input_thresh = 1.0; 
  
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Shell::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  if (param == "input_heading_var") {
    m_input_heading_var = toupper(val);
    return(true); 

  } else if (param == "input_speed_var") {
    m_input_speed_var = toupper(val);
    return(true); 

  }

  /*
  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "foo") && isNumber(val)) {
    // Set local member variables here
    return(true);
  }
  else if (param == "bar") {
    // return(setBooleanOnString(m_my_bool, val));
  }
  */
  
  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_Shell::onSetParamComplete()
{
  if (m_input_heading_var != "") 
    addInfoVars(m_input_heading_var);

  if (m_input_speed_var != "") 
    addInfoVars(m_input_speed_var);
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_Shell::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_Shell::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Shell::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_Shell::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_Shell::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_Shell::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_Shell::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;

  
  // Part 2: Get input values and check for staleness
  bool ok_heading, ok_speed;
  m_input_heading =  getBufferDoubleVal(m_input_heading_var, ok_heading);
  m_input_speed   =  getBufferDoubleVal(m_input_speed_var, ok_speed);

  if ( !ok_heading || !ok_speed ) {
    postEMessage("input heading or input speed not found");
    return(ipf);
  }
    

  double tstamp_heading = getBufferTimeVal(m_input_heading_var);
  double tstamp_speed = getBufferTimeVal(m_input_speed_var);
  
  if((tstamp_heading > m_stale_input_thresh) ||
     (tstamp_speed > m_stale_input_thresh)) {
    postEMessage("input heading or input speed info is stale");
    return(ipf);
  }

  // Part 3:  Build separate IvP functions for speed and heading
  ZAIC_SPD spd_zaic(m_domain, "speed");
  // setParams(double summit, double peakwidth, double basewidth, double summitdelta,
  //           double minutil, double maxutil
  spd_zaic.setParams(m_input_speed, 0.1, m_input_speed+0.4, 70, 20);
  
  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  if(!spd_ipf)
    postWMessage("Failure on the SPD ZAIC via ZAIC_PEAK utility");


  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(m_input_heading);
  crs_zaic.setBaseWidth(180);
  crs_zaic.setValueWrap(true);

  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);  
  if(!crs_ipf) 
    postWMessage("Failure on the CRS ZAIC");

  // Couple the two together
  OF_Coupler coupler;
  ipf = coupler.couple(crs_ipf, spd_ipf, 50, 50);
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

