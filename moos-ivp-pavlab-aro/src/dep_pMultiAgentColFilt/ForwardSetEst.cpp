/*****************************************************************/
/*    NAME: Tyler Paine                                          */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: FowardSetSet.cpp                                     */
/*    DATE: July 2024                                            */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include "ForwardSetEst.h"


using namespace std;

ForwardSetEst::ForwardSetEst()
{
  m_nominal_fwd_speed = 1.0;
  m_vname = "unset";

  m_yaw_rate_filt_time_const = 1.0/4.0;
  m_yaw_rate_last = 0.0;
  m_heading_last  = 0.0; 
  m_last_time     = 0.0;
  
}

ForwardSetEst::~ ForwardSetEst() {};


//----------------------------------------
//  Procedure setNominalFwdSetPoly(std::string)
bool ForwardSetEst::setNominalFwdSetPoly(string spec)
{
  m_nominal_fwd_poly = string2Poly(spec);
  return(m_nominal_fwd_poly.is_convex()); 
}

//------------------------------------------
//  Procedure setNominalFwdSetSpeed(double )
bool ForwardSetEst::setNominalFwdSetSpeed(double val){
  if (val <= 0.0)
    return(false);
  
  m_nominal_fwd_speed = val;
  return(true); 
}



//------------------------------------------
//  Procedure updateSet(double curr_x, double curr_y, double curr_hdg)
bool ForwardSetEst::updateSet(double curr_x, double curr_y, double curr_hdg,
			      double curr_spd, double time){

  // update m_curr_fwd_poly
  m_curr_fwd_poly = m_nominal_fwd_poly;

  // Part 1/3
  // scale based on speed;
  double speed_gain = curr_spd / m_nominal_fwd_speed;

  double x, y;
  for (unsigned int i = 0; i < m_curr_fwd_poly.size(); i++){
    x = m_curr_fwd_poly.get_vx(i);
    y = m_curr_fwd_poly.get_vy(i); 
    x *= speed_gain;
    y *= speed_gain;
    m_curr_fwd_poly.mod_vertex(i, x, y);
  }

  // Part 2/3
  // rotate according to heading and expected turn.

  // First calcuate the turn from the filtered yaw rate
  // which is calcuated using the heading.
  
  double dt = time - m_last_time;
  if (dt <= 0.0)
    dt = 1.0;
  
  double new_yaw_rate = (curr_hdg - m_heading_last) / dt;

  // filter it
  double alpha = 1.0 - exp( -1.0 * dt / m_yaw_rate_filt_time_const);
  m_yaw_rate_last +=  alpha * (new_yaw_rate - m_yaw_rate_last);

  // about how far in the future are we projecting?
  double delta_y = m_nominal_fwd_poly.get_max_y() - m_nominal_fwd_poly.get_min_y();
  double time_horizion = delta_y / m_nominal_fwd_speed;

  double total_est_rot = time_horizion * m_yaw_rate_last; // lots of assumptions

  double total_angle = curr_hdg + total_est_rot;

  // rotate about the origin. 
  m_curr_fwd_poly.rotate(total_angle, 0.0, 0.0); 


  // Part 3/3
  // Translate this polygon into the global coordinate system
  m_curr_fwd_poly.shift_horz(curr_x);
  m_curr_fwd_poly.shift_vert(curr_y);
  
  
  return(true); 
}


//------------------------------------------
//  Procedure willCollide(XYPolygon & other_agent_set) 
bool ForwardSetEst::willCollide(XYPolygon & other_agent_set) const{
  return(m_curr_fwd_poly.intersects(other_agent_set));  
}


//------------------------------------------------
// Procedure getSpec()
std::string ForwardSetEst::getSpec(std::string color){
  m_curr_fwd_poly.set_color("edge", color);
  m_curr_fwd_poly.set_color("vertex", color);
  m_curr_fwd_poly.set_label(m_vname);

  return(m_curr_fwd_poly.get_spec(4)); 

}


//--------------------------------------------------
// Procedure isSpecValid()
bool ForwardSetEst::isSpecValid(){


  std::string spec = m_curr_fwd_poly.get_spec(4);
  XYPolygon new_poly = string2Poly(spec);

  if (new_poly.area() < 0.1)
    return(false);
  
  if((new_poly.size()==0) && new_poly.active())
    return(false);
  else
    return(true); 
}

