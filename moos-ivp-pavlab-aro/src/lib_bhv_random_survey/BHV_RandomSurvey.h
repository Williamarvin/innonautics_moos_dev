/*****************************************************************/
/*    NAME: Filip Stromstad                                      */
/*    ORGN: Dept of Mechanical Eng, MIT Cambridge MA             */
/*    FILE: BHV_RandomSurvey.h                                   */
/*    DATE: May 20th 2024                                        */
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
 
#ifndef BHV_RANDOM_SURVEY_HEADER
#define BHV_RANDOM_SURVEY_HEADER

#include <string>
#include "IvPBehavior.h"
#include "XYPoint.h"
#include "XYPolygon.h"

class BHV_RandomSurvey : public IvPBehavior {
public:
  BHV_RandomSurvey(IvPDomain);
  ~BHV_RandomSurvey() {};
  
  bool         setParam(std::string, std::string);
  void         onIdleState();
  IvPFunction* onRunState();
  void         onEveryState(std::string);
  void         onIdleToRunState();
  void         onRunToIdleState();

  
protected:
  IvPFunction* buildFunction();
  void         updateWaypoint();
  void         postViewPoint(bool viewable=true);

protected: // State variables
  double   m_osx;
  double   m_osy;
  double   m_curr_time;

  double   m_ptx;
  double   m_pty;
  bool     m_pt_set;

  XYPolygon m_survey_region;

protected: // Config variables
  double m_capture_radius;
  double m_desired_speed;
};

#define IVP_EXPORT_FUNCTION
extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_RandomSurvey(domain);}
}
#endif
