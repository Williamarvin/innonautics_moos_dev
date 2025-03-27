/************************************************************/
/*    NAME: Raymond Turrisi                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Fig8.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef Fig8_HEADER
#define Fig8_HEADER

#include <string>
#include "IvPBehavior.h"
#include "NodeRecord.h"
#include <cstdarg>
#include "lemniscate.hpp"

class BHV_Fig8 : public IvPBehavior
{
public:
  BHV_Fig8(IvPDomain);
  ~BHV_Fig8() {};

  bool setParam(std::string, std::string);
  void onSetParamComplete();
  void onCompleteState();
  void onIdleState();
  void setDebug();
  void initStateVars();
  bool dbg_print(const char *format, ...);
  void onHelmStart();
  void postConfigStatus();
  void postErasables();
  void onRunToIdleState();
  void onIdleToRunState();
  void setInfoVars();
  void updateMail();
  void updateOwnshipNodeRecord();
  void updateParameters();
  IvPFunction *onRunState();
  void applyParamModifier(const string &update);

protected: // Local Utility functions

IvPFunction *getSimpleSpeedPeak(double desired_speed);
IvPFunction *getSimpleHeadingPeak(double desired_heading);
IvPFunction *getSimpleCoupledPeak(double desired_speed, double desired_heading);

protected: // Configuration parameters
  bool p_debug;
  FILE *m_cfile;
  std::string m_debug_fname;

protected: // State variables

  Lemniscate m_lemniscate_calculator;
  double m_osx;
  double m_osy;
  double m_osh;
  double p_lem_center_x;
  double p_lem_center_y;
  double p_lem_width;
  double p_lem_height;
  double p_lem_alpha_deg;
  double p_lem_alpha_rad;
  double p_lem_direction;
  double p_capture_radius;
  double p_slip_radius;
  double p_desired_speed;
  string p_updates_var;
  uint32_t p_lem_n_points;


  uint32_t m_lem_cidx;
  double m_lem_percent;
  double m_desired_heading;

  double p_max_cycles;
  double p_max_duration;
  double m_completed_cycles;
  double m_start_time;
  bool m_duration_exceeded;
  bool m_cycles_exceeded;
  
  double m_target_x;
  double m_target_y;

  NodeRecord m_os_node_record;
  double m_nr_t;
  double m_nr_t_prev;
  string m_os_color;
  string m_us_name;
  XYSegList m_latest_lemniscate_seglist;

  bool m_first;
};

#define IVP_EXPORT_FUNCTION

extern "C"
{
  IVP_EXPORT_FUNCTION IvPBehavior *createBehavior(std::string name, IvPDomain domain)
  {
    return new BHV_Fig8(domain);
  }
}
#endif
