/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_SearchControl.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef SearchControl_HEADER
#define SearchControl_HEADER


#include <string>
#include <list>
#include "XYPolygon.h"
#include "Odometer.h"
#include "IvPBehavior.h"

class BHV_SearchControl : public IvPBehavior {
public:
  BHV_SearchControl(IvPDomain);
  ~BHV_SearchControl() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  double getMinRadius();
  IvPFunction* onRunState();
protected: // Local Utility functions
 bool  updateOwnshipPosition();

 IvPFunction *buildOF();

 bool handleConfigOpRegion(std::string);
protected: // Configuration parameters
XYPolygon m_op_region;
protected: // State variables
XYPolygon m_proxonoi_region;

  Odometer  m_odometer;
  double m_region_center_x;
  double m_region_center_y;
  double m_region_radius;
  double m_osx;
  double m_osy;
  bool m_voronoi;
  bool   m_ownship_in_region;
  bool m_no_heading;
  bool m_outbound;
  double m_rand_heading;
  bool m_stoch;
  bool m_rand_speed;
  double m_max_x;
  double m_max_y;
  double m_cruise_speed;
  double m_spin_rad;
  double m_max_speed;
  int m_in;
  int m_short;
  int m_ran;
  std::string m_mode;
protected: // Local Utility functions

protected: // Configuration parameters

protected: // State variables
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_SearchControl(domain);}
}
#endif
