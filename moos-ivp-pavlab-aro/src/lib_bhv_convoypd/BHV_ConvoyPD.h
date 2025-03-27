
/* **************************************************************
  NAME: Raymond Turrisi
  ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
  FILE: BHV_ConvoyPD.cpp
  CIRC: November 2023
  DESC:
    General PD
    --
    This is a convoying behavior which uses a PD-esk controller
    for maintaining a speed and heading within a desired follow distance.
    An exhaustive description will be provided elsewhere. At its
    core, this behavior runs on all agents in a convoy, and produces different
    behavior for whether or not the agent is THE leader of the convoy,
    or is a a follower in the convoy. The leader seeds encoded points defining
    a 'trajectory' for all the agents to follow. These encoded points
    contain state information for the leader at the time in which it seeds the point.
    A follower agent follows this trajectory, while trying to match the leaders
    state at the time in which the point was seeded, i.e. if an agent is generally
    lagging from its desired follow distance, a strength in the P component of the
    controller increases the speed in which the agent pursues its target,
    however if the leader was taking a turn and slowed its speed along this trajectory
    the agent will slow down on this turn to prevent overshoot or large
    deviations from the trajectory. Imagine virtual springs and dampers, where one
    directional springs connect an agent to its target along a trajectory,
    and a virtual damper bound between an agent speed and the leaders speed
    at that point in the trajectory.

    Algorithmically, the leader is seeding/encoding and sending the point to its follower,
    where this point is added to the followers queue. While it captures points in this
    queue, points are passed sequentially to its follower until it reaches the tail of the
    convoy, and is then destroyed.

    Due to frequent message drops in simulation/on the water, there are redundant
    broadcasting protocol, in which all agents certain expected information to fill
    in parts of what an agent may have missed.

    PD Synchronization
    --
    Coming soon

  LICENSE:
    This is unreleased BETA code. No permission is granted or
    implied to use, copy, modify, and distribute this software
    except by the author(s), or those designated by the author.
************************************************************** */

#ifndef ConvoyPD_HEADER
#define ConvoyPD_HEADER

#include <string>
#include "IvPBehavior.h"
#include <list>
#include <map>
#include "ConvoyPointQueue.h"
#include "AgentInfo.h"
#include <cstdarg> //va_list, va_start, va_end

class BHV_ConvoyPD : public IvPBehavior
{
public:
  BHV_ConvoyPD(IvPDomain);
  ~BHV_ConvoyPD(){};

  bool setParam(std::string, std::string);
  void onSetParamComplete();
  void onCompleteState();
  void onIdleState();
  void onHelmStart();
  void postConfigStatus();
  void onRunToIdleState();
  void onIdleToRunState();

  IvPFunction *onRunState();

protected: // Local Utility functions
  //Incoming

  /**
   * @brief Updates/observes changes to all incoming messages
  */
  void updateMessages();

  /**
   * @brief Updates the agent info for the agent with the given name
   * 
   * @param name 
   */
  void updateAgentInfo(std::string name);

  /**
   * @brief Updates our ownship state information which we are tracking internally
   * 
   */
  void updateOwnshipState();

  /**
   * @brief If we are following points in the convoy point queue, we determine 
   * if we have captured the point or if the point is outside our capture radius
   * but meets slip conditions
   * 
   */
  void updateCapturePoint();

  /**
   * @brief Update if we are the leader
   * 
   */
  void updateIsLeader();

  /**
   * @brief If we receive an ordering posting from another agent which is more complete
   * than our observations, we update our ordering
   * 
   */
  void updateExtOrdering();

  /**
   * @brief If the contact manager is running and posting contacts, we update our contact list
   * to use redundant information for error checking
   * 
   */
  void updateContactList();

  /**
   * @brief When a bid is won, it tells us whom we are following, or whom another agent is following. With this,
   * we update whom we are following and also broadcast it for others in the convoy to reference. 
   * 
   */
  void updateCheckForContact();

  /**
   * @brief When we receive a lead point from whom we are following, we add it to our
   * convoy point queue
   * 
   */
  void updateLeadPoint();

  /**
   * @brief If we obtain new information on the ordering of agents in the convoy, we 
   * update our follower to leader/leader to follower mappings. 
   * 
   */
  void updateFtoLMapping();

  /**
   * @brief If we receive an update in real time to our udpates variable, we handle it
   * 
   */
  void updateUpdatesVar();

  /**
   * @brief If we receive a node report, we extract the information we need from it. 
   * Currently, we are only using a node report for obtaining our vehicle color. 
   * Probably a better source for this. 
   * 
   */
  void handleNodeReport();

  // Outgoing

  //// Calls all repeated state messages (state updates and broadcasts)
  
  /**
   * @brief This posts all perpetual updates for a run/idle cycle, calling
   * all other update functions and general broadcast functions
   * 
   */
  void postStateMessages();

  /**
   * @brief Herein we serialize and broadcast our realtime agent information, related
   * to managing the convoy
   * 
   */
  void postAgentInfo();

  /**
   * @brief A utility function for posting a labeled point above our vehicle
   * for communicating our distance error through pMarineViewer
   * 
   * @param err 
   */
  void postDistError(double err);

  /**
   * @brief If we are the leader, we broadcast this update for other agents
   * 
   */
  void postLeadership();

  /**
   * @brief We post the ordering incase other agents have incomplete information
   * due to dropped messages which were not recovered
   * 
   */
  void postOrdering();

  /**
   * @brief When we capture a point, we remove the graphic from pMarineViewer
   * while also passing the point to the follower agent
   * 
   * @param prv_cp 
   */
  void propagatePoint(ConvoyPoint prv_cp);

  /**
   * @brief For collective information which must be shared among all agents, 
   * which is built on messages which may be dropped, we perpetually broadcast
   * the complete information for other agents
   * 
   */
  void generalBroadcasts();

  /**
   * @brief If we are the leader, we encode a convoy point with our state information
   * and send it to our follower
   * 
   */
  void seedPoints();

  // Utilities

  /**
   * @brief Bound a value between a minimum and maximum value
   * 
   * @param value Value to be bounded
   * @param min 
   * @param max 
   */
  void clamp(double &value, double min, double max);

  /**
   * @brief Positive mod - in C++ you can get a negative mod value, however we 
   * want a strictly positive remainder value
   * 
   * @param a Dividend
   * @param b Divisor
   * @return double Remainder
   */
  double pmod(double a, double b);

  /**
   * @brief We schedule broadcasts on a common interval, and if we go through our cycles 
   * beyond this rate, we check to see whether or not we should repost a message
   * 
   * @param field Message type to query
   * @return true We exceeded our update cycle and should post an update
   * @return false We do not need to post an update yet
   */
  bool shouldRepost(std::string field);

  /**
   * @brief If we just posted an update to a broadcast message, we now schedule 
   * a repost for this message type
   * 
   * @param field Message type which was queried and should be rescheduled
   */
  void scheduleRepost(std::string field);

  /**
   * @brief A helper function for debugging the behavior during runtime. 
   * Works as vfprintf and prints to a commonfile or streamed output. When 
   * the behavior is constructed a file is opened, and whenever you want to 
   * obtain a debug print query, you format your message as you would with printf
   * and this opens the file, writes your message, and then closes the file. 
   * 
   * @param format 
   * @param ... 
   * @return true Successful write
   * @return false Unsuccessful write
   */
  bool dbg_print(const char *format, ...);

protected: // Configuration parameters
  // Parameters
  bool m_is_leader, m_is_midship, m_is_tail;

  double m_desired_speed;
  double m_max_speed;
  double m_point_update_distance;
  double m_ideal_follow_range;
  double m_max_lag_length;  // Max allowable length from our target before we start making comprimises
  double m_full_stop_range; // Safety - how close can we get to our target before saying we need to stop moving
  double m_capture_radius;
  double m_slip_radius;
  bool m_coupled;

  //Ownship trajectory gains
  double m_kp_spd_1, m_kp_spd_2, m_kd_spd_1, m_kd_spd_2, ki_spd; // Speed gains
  double m_kp_hdg_1, m_kp_hdg_2, m_kd_hdg_1, m_kd_hdg_2, ki_hdg; // Heading Gains

  //Coupling gains
  double m_kpc_spd, m_kdc_spd; // Speed gains

  std::string m_des_spd_k;
  std::string m_point_update_dist_k;

protected: // State variables
  // Debug tools
  bool m_debug;
  FILE *m_cfile;
  std::string m_debug_fname;

  // Ownship state variables
  XYPoint m_ownship;
  double m_osx, m_osy, m_osh;
  double m_osx_prv, m_osy_prv, m_osh_prv;
  double m_osx_tprv, m_osy_tprv, m_osh_tprv;
  double m_speed, m_osh_dot;
  double m_dist_err, m_spd_err;
  std::string m_color;

  // Contact state variables
  XYPoint m_target;
  std::string m_contact;
  AgentInfo m_self_agent_info;
  std::map<std::string, AgentInfo> m_contacts_lookup;
  std::string m_contact_list_str;
  std::vector<std::string> m_contact_list;
  std::string m_type_assignment;
  std::string m_task_state;

  // Broadcasts, udpates state variables, maintenance
  bool m_has_broadcast_contact;
  bool m_has_broadcast_leadership;
  double m_redudant_update_interval;
  std::map<std::string, double> m_next_redundant_update;
  XYPoint m_prev_err_point;

  // Maintenance variables
  double m_latest_buffer_time;
  double m_eps; // epsilon - an allowable error in meters
  double m_interval_odo;
  unsigned long int m_posted_points;
  ConvoyPointQueue m_cpq;

  // A collection of mappings for managing the ordering of the convoy
  std::map<std::string, std::string> m_follower_to_leader_mapping;
  std::map<std::string, std::string> m_leader_to_follower_mapping;
  std::vector<std::string> m_ordering_vector;
  std::string m_ordering_str;
  size_t m_place_in_convoy;
  std::string m_follower;

  // Message keys
  std::string m_nav_x_k;
  std::string m_nav_y_k;
  std::string m_nav_h_k;
  std::string m_nav_spd_k;
  std::string m_leader_k;
  std::string m_contact_k;
  std::string m_ext_ordering_k;
  std::string m_agent_info_k;
  std::string m_lead_point_k;
  std::string m_contact_list_k;
  std::string m_task_state_k;
  std::string m_nml_k;
  std::string m_nrl_k;
  std::string m_whotowho_k;
  std::string m_updates_var_k;
  std::string m_updates_buffer;
};

#define IVP_EXPORT_FUNCTION

extern "C"
{
  IVP_EXPORT_FUNCTION IvPBehavior *createBehavior(std::string name, IvPDomain domain)
  {
    return new BHV_ConvoyPD(domain);
  }
}
#endif
