/*****************************************************************/
/*    NAME: Michael Benjamin, Tyler Paine                        */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: EvalConvoyEngine.h                                   */
/*    DATE: July 25nd, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef FLD_EVAL_CONVOY_ENGINE_HEADER
#define FLD_EVAL_CONVOY_ENGINE_HEADER

#include <string>
#include <algorithm>  // for std::reverse
#include "ConvoyRecap.h"
#include "ConvoyStatRecap.h"
#include "ConvoySpdPolicy.h"
#include "ConvoyOrderDetector.h"

#include "NodeRecord.h"
#include "NodeMessage.h"
#include "NodeMessageUtils.h"
#include <math.h>  

class FldEvalConvoyEngine
{
 public:
   FldEvalConvoyEngine();
  ~FldEvalConvoyEngine() {};

  void setCurrTime(double);
  bool setParam(std::string, std::string);
  void updateMetrics();
  
  bool handleStatRecap(std::string);
  bool handleRecap(std::string);
  bool handleSpdPolicy(std::string);
  bool handleStatRecapAlly(std::string);
  bool handleNodeReport(std::string);

  bool   getBool(std::string) const;
  double getDouble(std::string) const;
  unsigned int getUInt(std::string) const;

  std::string getStrBool(std::string) const;
  std::string getStrDouble(std::string, int v=2) const;
  std::string getStrUInt(std::string) const;
  std::string getStrString(std::string) const;

  std::string getRecapSpec() const {return(m_recap.getSpec());}
  std::string getStatRecapSpec() const  {return(m_stat_recap.getSpec());}
  std::string getSpdPolicyTerse() const {return(m_spd_policy.getTerse());}

  std::vector<std::string> buildReport();

  std::string getConvoyReport();
  double      getMaxPairwiseRange() const;
  
protected: 
  void updateCoreMetrics();
  void updateRanges();
  bool calculateConvoyLength();
  double getRunningAvg(std::string vname, double val, double time);

 private: // Configuration variables
  std::string m_recap_var;
  std::string m_stat_recap_var;
  std::string m_spd_policy_var;

 private: //Internal State variables
  ConvoyRecap     m_recap;
  ConvoyStatRecap m_stat_recap;
  ConvoySpdPolicy m_spd_policy;
  ConvoyOrderDetector m_order_detector;

  std::map<std::string, NodeRecord> m_map_records;
  
  //  Range calculation values
  //  Stores the latest vehicle range data
  std::map<std::string, double> m_latest_vehicle_range;
  std::map<std::string, double> m_latest_vehicle_range_delta;

  // Stores the maximum vehicle range data
  std::map<std::string, double> m_max_vehicle_range_delta;
  std::map<std::string, double> m_max_vehicle_range_delta_time;

  // Variables used to detect and  calculate the oscillation feq.
  std::map<std::string, double> m_prev_vehicle_range_delta_val;
  std::map<std::string, double> m_prev_vehicle_range_delta_peak_time;
  std::map<std::string, double> m_prev_vehicle_range_delta_peak_val;
  std::map<std::string, double> m_vehicle_range_delta_peak_freq;

  // Variables used to calculate the running average for each vehicle
  std::map< std::string, std::list<double> > m_range_delta_hist;
  std::map< std::string, std::list<double> > m_range_delta_hist_tstamp;
  
  
  unsigned int m_recap_rcvd;
  unsigned int m_stat_recap_rcvd;
  unsigned int m_spd_policy_rcvd;
  
  double m_curr_time;
  double m_prev_time;
  double m_tstamp_first_recap;

  double m_convoy_length;

  ////////////////////////////
  // Hydroman calculations //
  std::map< std::string, double > m_gt_x;
  std::map< std::string, double > m_gt_y;
  std::map< std::string, double > m_nav_x_hydro;
  std::map< std::string, double > m_nav_y_hydro;

  std::map< std::string, double > error_x;
  std::map< std::string, double > error_y;
};

#endif 
