/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT                                             */
/*    FILE: BHV_NavCheck.cpp                                */
/*    DATE: Jun 11th, 2019                                  */
/*    DATE: Oct 19th, 2022 Mods to use improved conventions */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "VarDataPairUtils.h"
#include "BHV_NavCheck.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_NavCheck::BHV_NavCheck(IvPDomain domain) :
  IvPBehavior(domain)
{
  IvPBehavior::setParam("name", "navcheck");
  m_domain = subDomain(m_domain, "course,speed");

  // =====================================================
  // Init State vars
  m_nav_pos_age = -1;
  m_nav_spd_age = -1;
  m_nav_hdg_age = -1;
  
  // =====================================================
  // Init Config vars
  
  m_stale_nav_thresh = 5;
  m_stale_action  = "error";
  m_check_on_idle = true;
  
  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING, NAV_SPEED");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_NavCheck::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());

  bool handled = true;
  if(param == "stale_action") {
    if((val != "error") && (val != "warning"))
      handled = false;
    else
      m_stale_action = val;
  }
  else if(param == "stale_nav_thresh") 
    return(setNonNegDoubleOnString(m_stale_nav_thresh, val));
  else if(param == "check_on_idle") 
    return(setBooleanOnString(m_check_on_idle, val));
  else if(param == "bad_pos_flag") 
    return(addVarDataPairOnString(m_bad_pos_flags, val));
  else if(param == "bad_hdg_flag") 
    return(addVarDataPairOnString(m_bad_hdg_flags, val));
  else if(param == "bad_spd_flag") 
    return(addVarDataPairOnString(m_bad_spd_flags, val));

  return(handled);
}

//---------------------------------------------------------------
// Procedure: onIdleState()

void BHV_NavCheck::onIdleState()
{
  if(m_check_on_idle)
    checkNav();
}

//---------------------------------------------------------------
// Procedure: onRunState()

IvPFunction* BHV_NavCheck::onRunState()
{
  checkNav();
  return(0);
}

//---------------------------------------------------------------
// Procedure: checkNav()

void BHV_NavCheck::checkNav()
{
  bool nav_pos_ok = updateOSPos();
  if(!nav_pos_ok)
    postFlags(m_bad_pos_flags);

  bool nav_hdg_ok = updateOSHdg();
  if(!nav_hdg_ok)
    postFlags(m_bad_hdg_flags);

  bool nav_spd_ok = updateOSSpd();
  if(!nav_spd_ok)
    postFlags(m_bad_spd_flags);
}

//-----------------------------------------------------------
// Procedure: updateOSPos()
//   Returns: true  If NAV_X/Y info is found and not stale
//            false Otherwise

bool BHV_NavCheck::updateOSPos(string fail_action) 
{
  bool   ok_osx = true;
  bool   ok_osy = true;
  double new_osx = getBufferDoubleVal("NAV_X", ok_osx);
  double new_osy = getBufferDoubleVal("NAV_Y", ok_osy);
  double age_osx = getBufferTimeVal("NAV_X");
  double age_osy = getBufferTimeVal("NAV_Y");

  bool all_ok = true;
  if(!ok_osy || !ok_osy)
    all_ok = false;
  if((age_osx > m_stale_nav_thresh) ||
     (age_osy > m_stale_nav_thresh))
    all_ok = false;

  if(!all_ok) {
    if(fail_action == "error")
      postEMessage("ownship NAV_X/Y not found or stale.");
    else if(fail_action == "warn")
      postWMessage("ownship NAV_X/Y not found or stale.");
    return(false);
  }

  // Age is the max of osx and osy age
  m_nav_pos_age = age_osx;
  if(age_osy > age_osx)
    m_nav_pos_age = age_osy;
  
  m_osx = new_osx;
  m_osy = new_osy;
  return(true);
}

//-----------------------------------------------------------
// Procedure: updateOSHdg()
//   Returns: true  If NAV_HEADING info is found and not stale
//            false Otherwise

bool BHV_NavCheck::updateOSHdg(string fail_action) 
{
  bool   ok_osh = true;
  double new_osh = getBufferDoubleVal("NAV_HEADING", ok_osh);
  double age_osh = getBufferTimeVal("NAV_HEADING");

  if(!ok_osh || (age_osh > m_stale_nav_thresh)) {
    if(fail_action == "error")
      postEMessage("ownship NAV_HEADING not found or stale.");
    else if(fail_action == "warn")
      postWMessage("ownship NAV_HEADING not found or stale.");
    return(false);
  }

  m_nav_hdg_age = age_osh;

  m_osh = new_osh;
  return(true);
}

//-----------------------------------------------------------
// Procedure: updateOSSpd()
//   Returns: true:  If NAV_SPEED info is found and not stale
//            false: Otherwise

bool BHV_NavCheck::updateOSSpd(string fail_action) 
{
  bool   ok_osv  = true;
  double new_osv = getBufferDoubleVal("NAV_SPEED", ok_osv);
  double age_osv = getBufferTimeVal("NAV_SPEED");
  
  if(!ok_osv || (age_osv > m_stale_nav_thresh)) {
    if(fail_action == "error")
      postEMessage("ownship NAV_SPEED not found or stale.");
    else if(fail_action == "warn")
      postWMessage("ownship NAV_SPEED not found or stale.");
    return(false);
  }

  m_nav_spd_age = age_osv;
  
  m_osv = new_osv;

  return(true);
}

