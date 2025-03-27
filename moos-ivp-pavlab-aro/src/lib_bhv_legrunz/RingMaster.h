/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng, MIT                          */
/*    FILE: RingMaster.h                                         */
/*    DATE: June 6th, 2023                                       */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/
 
#ifndef RING_MASTER_HEADER
#define RING_MASTER_HEADER

#include <string>
#include <map>
//#include "LegRun.h"

class RingMaster {
public:
  RingMaster();
  ~RingMaster() {}

  void setCurrUTC(double);

  void setDimensionsLR(std::string vname, double leg,
		       double t1, double t2);
  
  bool updatePosCN(std::string vname, double spd,
		   double degs, double utc);

  void   updateCenter();

  double getOwnDegs();
  double getOwnDegsPerSec();
  
  double getCenterDegs() {return(m_center_degs);}
  double getCenterDist() {return(m_center_dist);}

  double getGrpTurn1Len() const {return(m_glr_turn1_len);}
  double getGrpTurn2Len() const {return(m_glr_turn2_len);}
  double getGrpTotalLen() const {return(m_glr_total_len);}

  unsigned int getTotalStale() const {return(m_total_stale);}
  
private:
  void extrapolate();
  void checkForStale();
  void updateGroupDims();
  
protected: // Config vars
  double m_stale_thresh;
  bool   m_coord_extrap;
  
protected: // State Vars (positions)
  double m_curr_utc;  
  double m_center_dist;
  double m_center_degs;
  bool   m_guard_active;

  // Keyed on vname
  std::map<std::string, double> m_map_degs;
  std::map<std::string, double> m_map_spd;
  std::map<std::string, double> m_map_utc;

protected: // State Vars (dimensions)

  // latest dimensions from other vehcles' legruns
  std::map<std::string, double> m_map_td1;
  std::map<std::string, double> m_map_td2;
  std::map<std::string, double> m_map_leg;
  // latest avg dimensions from other vehicle's legruns
  double m_glr_total_len;
  double m_glr_turn1_len;
  double m_glr_turn2_len;
  double m_glr_leg_len;

protected: // State Vars (other)
  unsigned int m_total_stale;
  
};

#endif
