/************************************************************/
/*    NAME: Filip Stromstad                                 */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Demuster.h                                  */
/*    DATE: 03.13.2024                                      */
/************************************************************/

#ifndef Demuster_HEADER
#define Demuster_HEADER

#include "IvPBehavior.h"
#include <string>
#include <map>
#include "XYPoint.h"
#include "XYSegList.h"
#include "XYPolygon.h"

enum BlockStatus{
  UNBLOCKED,
  TEMP_BLOCKED,
  PHYSICALLY_BLOCKED,
};
//DEADLOCKED
//PERMANENTLY_BLOCKED


class BHV_Demuster : public IvPBehavior {
public:
  BHV_Demuster(IvPDomain);
  ~BHV_Demuster() {};
  
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
  bool fetchNewMail();

protected: // Configuration parameters
  // Configuration parameters - Dubin trajectory
  double    m_goal_heading;  
  double    m_goal_x;
  double    m_goal_y;
  double    m_r1;
  double    m_r2;
  double    m_r3;
  double    m_precision;
  XYPolygon m_op_region; //Operational region
  bool      m_visualize_path_idle;
  bool      m_project_first_point_by_capture_radius;
  bool      m_only_right_turns; //Only right turns allowed first turn

  // Configuration parameters - Waypoint following
  double m_capture_radius;
  double m_slip_radius;
  double m_drift_radius;
  double m_drift_heading_thresh;

  // Configuration parameters - Speed policy
  double m_speed_default;
  double m_speed_desired;
  double m_speed_previous; //For LPF
  double m_speed_LPF_alpha;

  double m_max_safety_dist;
  double m_min_safety_dist; 
  double m_max_horizon_meters;
  double m_min_horizon_meters;

  int    m_slowdown_range;
  bool   m_use_dynamic_speed;
  bool   m_synchronize;
  bool   m_turn_in_place;
  double m_reverse_thrust;

  std::string m_mode;
  int m_seq_counter;

  bool   m_show_visualization;

protected: // State variables
  // State variables - Ownship (os)
  double m_osx;
  double m_osy;
  double m_os_speed;
  double m_osh;
  double m_osh_comp;
  bool   m_use_compass_heading;

  // State variables - Current trajectory
  XYSegList m_trajectory;
  int       m_curr_ix;         //Index of next point
  double    m_current_cpa;     //Closest point of approach to next point
  XYPoint   m_nextpt;
  XYPoint   m_nextnextpt;
  XYPoint   m_lastnextpt;
  bool      m_path_generated;
  int       m_number_of_paths_generated;

  // State variables - Block matrix
  std::map<std::string, XYSegList> m_trajectory_map;
  std::map<std::string, double> m_speed_map;
  std::map<std::string, std::map<std::string, BlockStatus>> m_block_matrix;
  std::map<std::string, std::map<std::string, double>> m_temp_block_dist_matrix;
  double m_temp_block_limit;

  std::vector<std::string> m_handshakes;
  std::vector<std::string> m_nodes_in_proximity;
  double m_proximity_range;    
  std::map<std::string, std::string> m_known_traj_numbers;
  int m_os_trajectory_number;

  // State variables - Timers
  double m_potential_permablock_timer;
  double m_potential_deadlock_timer;
  double m_deadlock_timer;
};

// bool depthFirstSearch(std::string name, std::map<std::string, std::map<std::string, BlockStatus>> &block_matrix, std::vector<std::string> &visited, std::vector<std::string> &recursive_stack);
// bool isPermablocked(std::string name, std::map<std::string, std::map<std::string, BlockStatus>> &block_matrix, std::map<std::string, XYSegList> &trajectory_map);

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Demuster(domain);}
}
#endif
