/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng, MIT                          */
/*    FILE: BHV_LegRunZ.h                                        */
/*    DATE: May 30th, 2023                                       */
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
 
#ifndef BHV_LEG_RUNZ_HEADER
#define BHV_LEG_RUNZ_HEADER

#include <string>
#include "IvPBehavior.h"
#include "WaypointEngineX.h"
#include "XYPoint.h"
#include "Odometer.h"
#include "WrapDetector.h"
#include "HintHolder.h"
#include "RingMaster.h"
#include "SpdModulator.h"
#include "LegRun.h"

class BHV_LegRunZ : public IvPBehavior {
public:
  BHV_LegRunZ(IvPDomain);
  ~BHV_LegRunZ() {}
  
  bool        setParam(std::string, std::string);
  IvPFunction* onRunState();
  void        onIdleState();
  void        onRunToIdleState();
  void        onIdleToRunState();
  void        onSetParamComplete();
  void        onCompleteState() {eraseAllViewables();}
  void        postConfigStatus();
  std::string expandMacros(std::string);

protected:
  bool   handleConfigTurnParam(std::string, std::string);
  bool   handleConfigLegSpeed(std::string);
  bool   handleConfigCoord(std::string);

  bool   updateInfoIn();
  void   postStatusReport();

  void   updateSpdModulator();
  
  void   postLegSegList(bool active=true);
  void   postLegRing(bool active=true);
  void   postTurnSegList(bool active=true);
  void   postAllSegList(bool active=true);
  void   postSteerPoints(bool active=true);
  void   postLegPoints(bool active=true);
  void   postTurnPreview(bool active=true);
  void   postTurnPoints(bool active=true);
  void   eraseAllViewables();

  bool   initTurnPoints();
  void   checkProvisional();
  bool   checkValidLegRun();
  
  bool   onRunStateLegMode();
  bool   onRunStateTurnMode();

  void   setMode(std::string);
  void   initLegMode();
  void   handleModeSwitch();
  void   updateLegSpd();

  void   advanceModePending();
  
  IvPFunction* buildOF();

  void   updateGenLegRun();
  void   postLengthSpecs(bool force=false);
  void   postPositionStatus();
  void   setGrpDistFromBeg();
  void   setTurnCoordSpd(double);
  void   postGrpAvgPoint(double gpa, bool active=true);

  double rawDistFromP1(double gpa);

  double rawDistToP1(double gpa);
  double rawDistToP2(double gpa); 
  double glrDistToP1(double gpa);
  double glrDistToP2(double gpa);
  double distFromP1(double gpa, double, double, double);
  double distToP1(double gpa, double, double, double);
  double distToP2(double gpa, double, double, double); 
  
protected: 
  WaypointEngineX m_wpteng_turn;
  WaypointEngineX m_wpteng_legs;

  LegRun m_legrun;

  SpdModulator m_spd_modulator;
  
protected: // Evaluation tools
  Odometer     m_odometer;
  WrapDetector m_wrap_detector;
  
protected: // Config vars
  double   m_max_spd;
  double   m_cruise_spd;
  double   m_patience;  // [0,100]
  double   m_offboard_tgap;
  std::string   m_init_leg_mode; // ("fixed"), "close_turn", "far_turn"
  
  // Event flags unique to this behavior
  std::vector<VarDataPair> m_cycle_flags;
  std::vector<VarDataPair> m_wpt_flags;
  std::vector<VarDataPair> m_leg_flags;
  std::vector<VarDataPair> m_mid_flags;
  std::vector<VarDataPair> m_start_leg_flags;
  std::vector<VarDataPair> m_start_turn_flags;

  double m_mid_pct; 
  
  // Visual hints affecting properties of seglists/points
  HintHolder  m_hints;

  std::vector<double> m_leg_spds;        // config var
  int     m_leg_spds_ix;     // state  var
  double  m_leg_spds_curr;   // state  var
  bool    m_leg_spds_repeat; // config var
  bool    m_leg_spds_onturn; // config var

  std::string m_coord; 
  bool        m_coord_extrap; 
  bool        m_coord_onleg; 
  
protected: // State vars
  std::string  m_mode;
  std::string  m_mode_pending;
  XYPoint      m_nextpt;
  XYPoint      m_trackpt;
  XYPoint      m_turn_pt1;
  XYPoint      m_turn_pt2;
  XYPoint      m_prev_dpt;  
  bool         m_mid_event_yet;
  bool         m_preview_pending;
  unsigned int m_leg_count;
  unsigned int m_leg_count1;
  unsigned int m_leg_count2;

  bool         m_provisional;

protected: // State vars (turn_coord)

  // Support for multi-vehicle coordinated turning
  double m_turn_coord_spd; 

  RingMaster m_ring_master;
};


#define IVP_EXPORT_FUNCTION
extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior *createBehavior(std::string name,
						  IvPDomain domain) 
  {return new BHV_LegRunZ(domain);}
}
#endif
