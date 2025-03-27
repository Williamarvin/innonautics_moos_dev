/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_Convoy21.h                                       */
/*    DATE: April 4th, 2019                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/
 
#ifndef BHV_CONVOY_V21_HEADER
#define BHV_CONVOY_V21_HEADER

#include <string>
#include <list>
#include "VarDataPair.h"
#include "IvPContactBehavior.h"
#include "ConvoyMarker.h"
#include "ConvoySpdPolicy.h"
#include "MarkerTail.h"

class IvPDomain;
class BHV_ConvoyV21 : public IvPContactBehavior {
public:
  BHV_ConvoyV21(IvPDomain);
  ~BHV_ConvoyV21() {}

public: // Overloaded virtual functions
  IvPFunction* onRunState();
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onRunToIdleState();
  void         onIdleState() {};
  void         onParamUpdate(std::string);
  std::string  checkParamCollective();
  std::string  expandMacros(std::string);

protected:
  double getRelevance();
  double getPriority();
  IvPFunction *buildOF();

  bool   handleConfigHoldingPolicy(std::string);  
  bool   handleParamVisualHints(std::string);  
  bool   handleParamCompression(std::string);  
  void   handleNewContactSpd(double);

  void   drawMarker(ConvoyMarker, std::string color="");
  void   eraseMarker(ConvoyMarker);

  void   updateMetrics();

  bool   checkDropAftMarker();

  void   setCurrentMarker();
  void   clearMarkerTail();  

  void   postIdleRecap();
  void   postRecap(bool);
  void   postSpdPolicy();
  
protected: // State variables
  MarkerTail m_marker_tail;

  std::list<double> m_cn_spd_value;
  std::list<double> m_cn_spd_tstamp;
  
  double m_wptx;
  double m_wpty;
  double m_set_speed;
  
  double m_cnv_avg_2sec;
  double m_cnv_avg_5sec;

protected: // State variables (metrics)
  double m_convoy_range;
  double m_range_delta;
  double m_tail_range;
  double m_tail_angle;
  double m_marker_bng;
  double m_track_error;
  double m_alignment; 

  unsigned int m_reached_markers;
  unsigned int m_dropped_markers;

  unsigned int m_recap_index;
  unsigned int m_stat_recap_index;
  
private: // Configuration parameters
  double m_capture_radius;
  double m_slip_radius;
  double m_compression;
  bool   m_aft_patience;
  
  double m_patience; // [1,99]

  std::string m_holding_policy;
  
  double m_max_speed; 
  
  ConvoySpdPolicy m_spd_policy_base;
  ConvoySpdPolicy m_spd_policy;

  std::vector<VarDataPair> m_marker_flags;
  std::vector<VarDataPair> m_convoy_flags;

private: // Config Visual hints, output
  std::string m_hint_marker_color;
  std::string m_hint_marker_label_color;
  double      m_hint_marker_size;
  bool        m_post_recap_verbose;
};

#define IVP_EXPORT_FUNCTION
extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name,
						   IvPDomain domain)
  {return new BHV_ConvoyV21(domain);}
}
#endif


