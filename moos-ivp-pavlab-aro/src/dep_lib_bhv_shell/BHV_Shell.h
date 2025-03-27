/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Shell.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef Shell_HEADER
#define Shell_HEADER

#include <string>
#include "IvPBehavior.h"
#include "ZAIC_PEAK.h"
#include "ZAIC_SPD.h"
#include "OF_Coupler.h"

class BHV_Shell : public IvPBehavior {
public:
  BHV_Shell(IvPDomain);
  ~BHV_Shell() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

protected: // Local Utility functions

protected: // Configuration parameters

protected: // State variables
  std::string m_input_heading_var;
  std::string m_input_speed_var;

  double m_input_heading;
  double m_input_speed; 

  double m_stale_input_thresh; 
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Shell(domain);}
}
#endif
