/************************************************************/
/*    NAME: Filip Stromstad                                 */
/*    ORGN: MIT                                             */
/*    FILE: BHV_DubinsPath.h                                */
/*    DATE: 03.13.2024                                      */
/************************************************************/

#ifndef DubinsPath_HEADER
#define DubinsPath_HEADER

#include <string>
#include "IvPBehavior.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include "XYPolygon.h"


class BHV_DubinsPath : public IvPBehavior {
public:
  BHV_DubinsPath(IvPDomain);
  ~BHV_DubinsPath() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

private:
  void         updateConfigParameters();  

protected: // Local Utility functions
  IvPFunction* buildOF();

protected: // Configuration parameters
  std::string m_os_name;
  double m_goal_heading;  
  double m_goal_x;
  double m_goal_y;
  double m_r1;
  double m_r2;
  double m_r3;              //TODO: Check that these^ are given as input

  double m_precision;
  double m_speed_desired;
  double m_speed_previous; //For LPF
  double m_capture_radius;
  double m_slip_radius;
  double m_drift_radius;
  double m_drift_heading_thresh;
  bool   m_project_first_point_by_capture_radius;
  double m_speed_LPF_alpha;

  bool m_visualize_path_idle;
  int m_slowdown_range; //Start slowing down this many meters from goal

  XYPolygon m_op_region; //Operational region
  bool m_only_right_turns; //Only right turns allowed first turn

protected: // State variables
  double m_osx;
  double m_osy;
  double m_osh;
  double m_osh_comp; //Compass heading
  double m_compass_declination;
  bool m_use_compass_heading;

  XYSegList m_trajectory;
  XYPoint m_nextpt;
  XYPoint m_nextnextpt;
  XYPoint m_lastnextpt;
  int m_curr_ix;            //Index of next point
  double m_current_cpa;     //Closest point of approach to next point

  double m_optimal_path_length;

  bool m_path_generated;
  int m_number_of_paths_generated;

  double m_idle_time;
  bool m_deadlock; //TODO: Not needed anymore?
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_DubinsPath(domain);}
}
#endif
