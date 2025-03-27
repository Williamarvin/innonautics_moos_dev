/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_MoveToRegion.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef MoveToRegion_HEADER
#define MoveToRegion_HEADER

#include <string>
#include "IvPBehavior.h"
#include "XYPolygon.h"
#include "XYFormatUtilsPoly.h" // string2poly
#include "ZAIC_PEAK.h"
#include "ZAIC_SPD.h"
#include "AngleUtils.h"  // relAng()
#include "OF_Coupler.h"
#include "GeomUtils.h"  // distPointToPoint()


class BHV_MoveToRegion : public IvPBehavior {
public:
  BHV_MoveToRegion(IvPDomain);
  ~BHV_MoveToRegion() {};
  
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

  bool updateOwnshipPosition();
  bool handleConfigRegion(string str);
  bool updateSetPoint();
  void postSetPoint();
  bool handleConfigVisualHint(string hint);
  bool checkForCompletion(); 

protected: // Configuration parameters
  double m_stale_nav_thresh;
  double m_patience;

protected: // State variables
  double m_osx;
  double m_osy;
  double m_setpt_x;
  double m_setpt_y;
  double m_setpt_x_prev;
  double m_setpt_y_prev;
  
  double m_cruise_speed; 
  XYPolygon m_poly_region;
  unsigned int m_region_count; 
  
  bool        m_setpt_viewable;
  double      m_hint_setpt_size;
  std::string m_hint_setpt_color;
  std::string m_hint_setpt_lcolor;
  
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_MoveToRegion(domain);}
}
#endif
