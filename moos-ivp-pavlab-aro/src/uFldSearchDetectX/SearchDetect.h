/************************************************************/
/*    NAME: Mike Benjamin - derived from Craig Evans        */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SearchDetect.h                                  */
/*    DATE: Sep 22nd, 2022                                  */
/************************************************************/

#ifndef SEARCH_DETECT_X_HEADER
#define SEARCH_DETECT_X_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <map>
#include "CPAMonitor.h"
#include "CPAEventNew.h"
#include "VarDataPair.h"
#include "LogicCondition.h"
#include "InfoBuffer.h"

class SearchDetect : public AppCastingMOOSApp
{
 public:
   SearchDetect();
  ~SearchDetect() {};

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();


 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void handleMailNodeReport(std::string);
   void handleCPAEventNew(CPAEventNew event);
   void getEstimate();
   bool initRange(std::string);

 protected: // Config params
  bool        m_prob;
  std::string m_target_name;
  double      m_encounter_dist;
  double      m_sigma;

 protected: // State variables
  bool   m_hit;
  double m_p;
  double m_cpaunder;

  double m_rand_val;
  double m_numhit;

  double m_time_elapse;
  double m_time1;

  bool   m_mission_start;

  std::string m_first_report;
  std::string m_second_report;
  std::string m_target_report;

  
  std::vector<double> m_detection_dist;
  std::vector<double> m_more_than_one;
  std::vector<bool> m_first_detection;
  std::vector<int> m_detection_total;
  std::vector<int> m_detection_run;
  std::vector<std::string> m_current_vname;

  CPAMonitor m_cpa_monitor;

  std::map<std::string, unsigned int> m_map_vname_detections;
  std::vector<unsigned int> m_total_detections;
  
  std::string m_param_summary;
  std::vector<std::string> m_notified_vehicles;
};

#endif 
