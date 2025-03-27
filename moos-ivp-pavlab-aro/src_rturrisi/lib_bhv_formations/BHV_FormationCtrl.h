/************************************************************/
/*    NAME: Raymond Turrisi */
/*    ORGN: MIT                                             */
/*    FILE: BHV_FormationCtrl.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef FormationCtrl_HEADER
#define FormationCtrl_HEADER

#include <string>
#include "IvPBehavior.h"
#include "FormationGeometry.hpp"
#include "AgentStateInfo.h"
#include "NodeRecord.h"
#include <cstdarg> //va_list, va_start, va_end
#include <cassert>

class BHV_FormationCtrl : public IvPBehavior
{
public:
  BHV_FormationCtrl(IvPDomain);
  ~BHV_FormationCtrl(){};

  void initStates();

  bool setParallelFormationType(std::string formation_type, std::string params);

  bool setFormation(std::string, std::string);

  bool setParam(std::string, std::string);
  /*
    Parameters will be nested, i.e. formation_type = vee : {min_separation = 5, vee_angle = 45}
    Depending on the formation type, some default parameters will automatically be adjusted for practicality and safety (i.e. if not convoy, idf = 0, otherwise idf = 5)

  */
  void onSetParamComplete();
  void onCompleteState();
  /*
    On Idle state, we shouldn't have to do anything
  */
  void onIdleState();

  void setDebug();

  bool dbg_print(const char *format, ...);

  /*
    On helm start, we initialize all our variables
  */

  double sat(double r_sat, double lambda, double r);

  void onHelmStart();

  void setInfoVars();

  void postConfigStatus();
  void updateLeadAgentStates();
  void updateAgentStates();
  void updateFormationGeometry();

  void assertInvariant(bool condition, const char *format, ...);

  void updateOwnshipNodeRecord();

  void updateOwnshipState();

  void onRunToIdleState();
  /*
    When transitioning from running to idling, we reset behavior states so when things transition back or become reactivated, there is a clean slate as if it was reinitialized
  */
  void onIdleToRunState();

  /*
    During the run state, we check for all variables which may have been passed between agents.
    If we are in a parallel formation, we are only receiving the instantaneous errors from each agent, while observing the position of the lead agent, and self maintaining our point queue
    If we are in a serial formation, we are receiving the instantaneous errors from each agent, as well as listening for a new point to include in our point queue, sent from the agent whom we are following
  */
  void updateMail();

  void postUpdates();
  void postQueueLead();

  /*
    On a run state, we are following a parallel or serial formation strategy, which is left to
    that individual function, which maintains unique assets as well as shared assets between formations

    If we are in a parallel formation, the region of attraction for synchronization
    is at the back of the queue (length of the queue), while if we are in a serial formation
    it is centered around the desired follow range.

    While we are in a formation, we also distinguish between being a leader, the first follower, other followers, and the last agent
    The behavior only needs to be running on a lead agent if we are in a parallel formation, since it can alter maneuvering in order
    to maintain the cohesiveness of the formation by making sure we don't take too aggressive turns which may not be achievable

    For serial formations (i.e. convoying), we need not have this behavior active on the lead vehicle, since the first follower can keep can eye
    on the leader and encode their dynamics for the rest of the convoy

    While generally in a formation, each agent is following their set point and managing their point queue,
    and synchronizing with other agents when the region of attraction is saturated (i.e. they are close enough to the track point to
    want to synchronize in the first place)

  */
  IvPFunction *onRunState();

  IvPFunction *getSimpleSpeedPeak(double desired_speed);
  IvPFunction *getSimpleHeadingPeak(double desired_heading);
  IvPFunction *getSimpleCoupledPeak(double desired_speed, double desired_heading);

  IvPFunction *followerConvoyBehavior();
  IvPFunction *followerFormationBehavior();
  IvPFunction *followerIvPBehavior();
  IvPFunction *leaderIvPBehavior();

protected: // Local Utility functions
           // Sat function (lambda, Rs, and r)
protected: // Configuration parameters
private:   // State variables
  FormationGeometry m_formation_geomgr;
  std::map<std::string, std::string> m_formation_params;
  uint32_t m_num_agents;
  std::string m_formation_type;
  double m_desired_speed;

  bool m_debug;
  FILE *m_cfile;
  std::string m_debug_fname;

  int32_t m_pos_idx;

  double m_kp_spd_1, m_kp_spd_2, m_kd_spd_1, m_kd_spd_2; // Speed gains
  double m_kp_hdg_1, m_kp_hdg_2, m_kd_hdg_1, m_kd_hdg_2; // Heading Gains

  // Coupling gains
  double m_rs_spd, m_rs_hdg;                       // Region of attraction - saturation region for coupling strength to be 1
  double m_lambda_roa_spd_sr, m_lambda_roa_hdg_sr; // Gains for the region of attraction, for speed and heading, and the saturation regions

  double m_kpc_spd, m_kdc_spd; // Speed gains
  double m_kpc_hdg, m_kdc_hdg; // Heading Gains

  AgentStateInfo m_os_state;
  AgentStateInfo m_os_state_prev;
  NodeRecord m_os_node_record;

  double m_nr_t;
  double m_nr_t_prev;
  double m_latest_r_err;
  double m_latest_r_reference;

  double m_max_speed;

  // Latest desired states from current point observed in queue
  double m_desired_x_cp;
  double m_desired_y_cp;
  double m_desired_x_dot_cp;
  double m_desired_y_dot_cp;

  double m_desired_yaw_cp;
  double m_desired_yaw_dot_cp;
  double m_desired_surge_cp;
  
  
  std::map<std::string, AgentStateInfo> m_agent_states;
  std::map<uint16_t, std::string> m_idx_to_agent_name;
  std::map<std::string, uint16_t> m_agent_name_to_idx;

  CtrlPointQueue m_ctrl_point_queue;

  double m_capture_radius;
  double m_slip_radius;

  double m_min_ctrl_point_sep;
  double m_max_ctrl_queue_length;
  
  XYSegList m_prev_posting;

  
};

#define IVP_EXPORT_FUNCTION

extern "C"
{
  IVP_EXPORT_FUNCTION IvPBehavior *createBehavior(std::string name, IvPDomain domain)
  {
    return new BHV_FormationCtrl(domain);
  }
}
#endif
