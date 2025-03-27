
/* **************************************************************
  NAME: Raymond Turrisi
  ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
  FILE: AgentInfo.h
  CIRC: November 2023
  DESC:
    A serialization/deserialization class for 'Agent Information'
    which is related to convoying. Experimental and subject to change.
    Should not be used outside the scope of ConvoyPD or ConvoySync.

  LICENSE:
    This is unreleased BETA code. No permission is granted or
    implied to use, copy, modify, and distribute this software
    except by the author(s), or those designated by the author.
************************************************************** */
#pragma once
#include <armadillo>
#include <string>

class AgentStateInfo
{
public:
  std::string name;  // 0
  double time_se;    // 1
  std::string color; // 2
  int32_t id;       // 3

  // World frame states (position and orientation in the world frame)
  arma::vec::fixed<6> q;       // [x, y, z, roll, pitch, yaw]
  arma::vec::fixed<6> q_dot;   // [x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot]
  arma::vec::fixed<6> q_dotdot;// [x_dotdot, y_dotdot, z_dotdot, roll_dotdot, pitch_dotdot, yaw_dotdot]

  //r_err: the individually interpreted trajectory error, which is specifically a distance/odometry 
  double r_err;
  double r_reference; //Magnitude reference in order to manipulate synchronization strategies

  // Error information
  arma::vec::fixed<6> q_err;       // [x_err, y_err, z_err, roll_err, pitch_err, yaw_err]
  arma::vec::fixed<6> q_dot_err;   // [x_dot_err, y_dot_err, z_dot_err, roll_dot_err, pitch_dot_err, yaw_dot_err]
  arma::vec::fixed<6> q_dotdot_err;// [x_dotdot_err, y_dotdot_err, z_dotdot_err, roll_dotdot_err, pitch_dotdot_err, yaw_dotdot_err]
  
  // Body frame velocities (linear and angular)
  arma::vec::fixed<6> nu;       // [u, v, w, p, q, r]
  arma::vec::fixed<6> nu_dot;   // [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot]

  // Error information
  arma::vec::fixed<6> nu_err;       // [u_err, v_err, w_err, p_err, q_err, r_err]
  arma::vec::fixed<6> nu_dot_err;   // [u_dot_err, v_dot_err, w_dot_err, p_dot_err, q_dot_err, r_dot_err]

  AgentStateInfo();

  AgentStateInfo(std::string strrep);

  AgentStateInfo(const AgentStateInfo &asi);

  std::string repr(std::string delim = ",");
};
