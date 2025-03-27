/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_VectorField.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef VectorField_HEADER
#define VectorField_HEADER

#include <string>
#include <list>
#include "XYPolygon.h"
#include "Odometer.h"
#include "IvPBehavior.h"

class BHV_VectorField : public IvPBehavior {
public:
  BHV_VectorField(IvPDomain);
  ~BHV_VectorField() {};
  
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
  bool   m_ownship_in_region;
  double m_max_x;
  double m_max_y;
  double m_cruise_speed;
  double m_spin_rad;
  int m_in;
  int m_short;
  bool m_got_dist;
  double m_dis;
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_VectorField(domain);}
}
#endif
