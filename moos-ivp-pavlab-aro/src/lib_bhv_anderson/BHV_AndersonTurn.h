/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_AndersonTurn.h                                   */
/*    DATE: Jul 26th, 2020                                       */
/*    DATE: Oct 21st, 2021 Major restructure                     */
/*****************************************************************/
 
#ifndef BHV_ANDERSON_TURN_HEADER
#define BHV_ANDERSON_TURN_HEADER

#include <vector>
#include "IvPBehavior.h"
#include "XYSegList.h"

class BHV_AndersonTurn : public IvPBehavior {
public:
  BHV_AndersonTurn(IvPDomain);
  ~BHV_AndersonTurn() {}
  
  bool        setParam(std::string, std::string);
  void        onSetParamComplete();
  IvPFunction* onRunState();
  void        onIdleState() {updateInfoIn();}
  void        onIdleToRunState();
  void        onRunToIdleState();
  std::string expandMacros(std::string);

 protected:
  bool         updateInfoIn();
  void         checkForEngageMsg();
  void         resetStateVars();
  IvPFunction* buildOF();

  double  headingDelta(double, double);
  double  angleDiffClock(double, double);
  double  angleDiffCounterClock(double, double);
  void    genTurnPath();
  void    drawTurnPath();
  void    eraseTurnPath();
  void    setMarkWaypt(bool=false);
  void    handleComplete();

 protected: // Configuration variables
  double  m_peakwidth;
  double  m_basewidth;
  double  m_summitdelta;

  // Set Default turn characteristics
  double  m_default_turn_thresh;
  double  m_default_capture_radius;
  double  m_default_turn_radius;
  unsigned int m_default_turn_points;
  
  // Prevailing turn characteristics: May be overridden with an
  // incoming engage message. Otherwise defaults are used.
  double  m_turn_thresh;
  double  m_capture_radius;
  double  m_turn_radius;
  unsigned int m_turn_points;

 protected: // State variables
  double  m_osx;
  double  m_osy;

  double  m_kx;
  double  m_ky;
  double  m_wptx;
  double  m_wpty;
  
  double  m_prev_hdg;
  double  m_curr_hdg;
  double  m_curr_spd;

  double  m_mark_hdg;
  double  m_mark_spd;
  double  m_all_turn;
  
  std::string m_state; // waiting, port, star

  std::vector<XYPoint> m_turn_path;

  std::string m_engage_var;
  
  unsigned int m_wix;
  
};

// Windows needs to explicitly specify functions to export from a dll
#ifdef WIN32
#define IVP_EXPORT_FUNCTION __declspec(dllexport) 
#else
#define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_AndersonTurn(domain);}
}
#endif
