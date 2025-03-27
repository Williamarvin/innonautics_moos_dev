/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT                                             */
/*    FILE: BHV_NavCheck.h                                  */
/*    DATE: Oct 19th, 2022                                  */
/************************************************************/

#ifndef BHV_NAV_CHECK_HEADER
#define BHV_NAV_CHECK_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_NavCheck : public IvPBehavior {
public:
  BHV_NavCheck(IvPDomain);
  ~BHV_NavCheck() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete() {};
  void         onCompleteState() {};
  void         onIdleState();
  void         onHelmStart() {};
  void         postConfigStatus() {};
  void         onRunToIdleState() {};
  void         onIdleToRunState() {};
  IvPFunction* onRunState();

protected: // Local Utility functions

  void checkNav();
  
  bool updateOSPos(std::string fail_action="error");
  bool updateOSHdg(std::string fail_action="error");
  bool updateOSSpd(std::string fail_action="error");
  
protected: // Config vars

  double m_stale_nav_thresh;
  string m_stale_action;
  bool   m_check_on_idle;
  
protected: // State vars

  double m_nav_pos_age;
  double m_nav_hdg_age;
  double m_nav_spd_age;

  bool   m_nav_pos_ok;
  bool   m_nav_hdg_ok;
  bool   m_nav_spd_ok;
  
  // Event flags unique to this behavior
  std::vector<VarDataPair> m_bad_pos_flags;
  std::vector<VarDataPair> m_bad_hdg_flags;
  std::vector<VarDataPair> m_bad_spd_flags;
};


// Windows needs to explicitly specify functions to export from a dll
#ifdef WIN32
#define IVP_EXPORT_FUNCTION __declspec(dllexport) 
#else
#define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_NavCheck(domain);}
}

#endif
