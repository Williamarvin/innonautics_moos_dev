/*****************************************************************/
/*    NAME: Tyler Paine                                          */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: FowardSetSet.h                                       */
/*    DATE: July 2024                                            */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef FORWARDSETEST_HEADER
#define FORWARDSETEST_HEADER

#include "MBUtils.h"

#include <string>
#include <list>
#include <math.h>   // for fabs
#include "XYPolygon.h"         // for forward poly
#include "XYFormatUtilsPoly.h" // for string2poly

class ForwardSetEst
{
public:
  ForwardSetEst();
  ~ForwardSetEst();

  // SET functions:

  bool setNominalFwdSetPoly(std::string spec);
  bool setNominalFwdSetSpeed(double val);
  void setName(std::string name) {m_vname = name; }

  // GET functions
  bool updateSet(double curr_x, double curr_y, double curr_hdg,
		 double curr_spd, double time);
  bool willCollide(XYPolygon & other_agent_set) const;
  
  XYPolygon getCurrFwdReachSet() {return(m_curr_fwd_poly);} 

  std::string getSpec(std::string color="white"); 
  std::string getSpecInactive()  {return(m_curr_fwd_poly.get_spec_inactive());}

  
  bool   isSpecValid();
  double area() {return(m_curr_fwd_poly.area());}
  
protected:

private:
  std::string m_vname;
  
  XYPolygon  m_nominal_fwd_poly;
  double     m_nominal_fwd_speed;

  double  m_yaw_rate_filt_time_const;
  double  m_yaw_rate_last;
  double  m_heading_last;
  double  m_last_time;

  XYPolygon  m_curr_fwd_poly; 
  
};

#endif


