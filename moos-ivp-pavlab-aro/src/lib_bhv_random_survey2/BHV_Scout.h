/*****************************************************************/
/*    NAME: T. Paine adapted from 2.680 work by M.Benjamin,      */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_Scout.h                                          */
/*    DATE: April 30th 2022                                      */
/*                                                               */
/* This program is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation; either version  */
/* 2 of the License, or (at your option) any later version.      */
/*                                                               */
/* This program is distributed in the hope that it will be       */
/* useful, but WITHOUT ANY WARRANTY; without even the implied    */
/* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/* PURPOSE. See the GNU General Public License for more details. */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with this program; if not, write to the Free    */
/* Software Foundation, Inc., 59 Temple Place - Suite 330,       */
/* Boston, MA 02111-1307, USA.                                   */
/*****************************************************************/
 
#ifndef BHV_SCOUT_HEADER
#define BHV_SCOUT_HEADER

#include <string>
#include "IvPBehavior.h"
#include "XYPoint.h"
#include "XYPolygon.h"

class BHV_Scout : public IvPBehavior {
public:
  BHV_Scout(IvPDomain);
  ~BHV_Scout() {};
  
  bool         setParam(std::string, std::string);
  void         onIdleState();
  IvPFunction* onRunState();
  void         onEveryState(std::string);
  void         onRunToIdleState(); 
  
protected:
  IvPFunction* buildFunction();
  void         updateScoutPoint();
  void         postViewPoint(bool viewable=true);
  void         postViewPolygon(bool viewable=true);
  

protected: // State variables
  double   m_osx;
  double   m_osy;
  double   m_curr_time;

  double   m_ptx;
  double   m_pty;
  bool     m_pt_set;
  bool     m_poly_posted; 

  XYPolygon m_rescue_region;

protected: // Config variables
  double m_capture_radius;
  double m_desired_speed;

  std::string m_tmate;

  std::string m_region_update_var;

  std::string m_edge_color;
  std::string m_vertex_color;
  std::string m_point_color;
  std::string m_label_prefix; 
  
};

#define IVP_EXPORT_FUNCTION
extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Scout(domain);}
}
#endif
