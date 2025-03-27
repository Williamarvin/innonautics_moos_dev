/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: FldBloomStormSim.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef FldBloomStormSim_HEADER
#define FldBloomStormSim_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYFormatUtilsPoly.h"
#include "XYPolygon.h"
#include <cstdlib>    // rand
#include "GeomUtils.h" // randPointInPoly
#include <queue>       
//#include "XYRangePulse.h"
#include <unistd.h>
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "XYCircle.h"

class FldBloomStormSim : public AppCastingMOOSApp
{
 public:
   FldBloomStormSim();
   ~FldBloomStormSim();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();
   bool handleMailNodeReport(const std::string& node_report_str);
   bool handleSampleFinishedLog(std::string vname);
   bool handleSampleFinishedLogOld(const std::string vname); 

 protected:
   void registerVariables();
   bool handleConfigRegion(std::string opstr, unsigned int number);
   
   
   bool manageBlooms();
   bool checkActiveBlooms();
   bool generateNewBloom();
   bool moveBlooms(); 

   
   bool notifyVehiclesOfBloom();
   
   bool manageStorm();
   bool checkActiveStorm(); 
   bool notifyVehiclesOfStorm();
   void postEval();

   double getRange(const XYCircle & Pulse, double time); 

 private: // Configuration variables
   XYPolygon m_region_1;
   XYPolygon m_region_2;
   XYPolygon m_storm_region;

   XYCircle m_storm_circle;

   double m_bloom_rad_max;
   double m_bloom_rad_min;
   double m_bloom_duration_max;
   double m_bloom_duration_min;
   double m_bloom_max_speed; 
   double m_probability_of_new_bloom;
   double m_time_between_bloom_attempts;
   double m_sensor_decay_parameter;

   bool   m_allow_storms;
   double m_storm_speed;
   double m_storm_radius; 
   double m_storm_max_angle;
   double m_time_between_storms;

 private: // State variables

   double m_last_storm_time;
   double m_last_bloom_start_time;
   double m_last_iteration_time; 
   
   double m_mixed_strat_run_avg;
   double m_last_eval_time;
   double m_mixed_strat_total_running_time;
   bool m_deployed;
   
   unsigned int m_bloom_count;
   unsigned int m_storm_count;
   unsigned int m_undetected_and_unsampled_blooms_completed;
   unsigned int m_sampled_blooms_completed;
   unsigned int m_detected_blooms_completed;

   std::vector<XYCircle> m_active_blooms;
   std::vector<double> m_bloom_vx;
   std::vector<double> m_bloom_vy;
   
   double m_storm_heading;
   double m_storm_x;
   double m_storm_y;
   double m_storm_last_update_time;
   bool m_storm_active;
   bool m_only_post_in_region_one;
   bool m_eval_started;

  // Key for each map below is the vehicle name. 
  std::map<std::string, NodeRecord>   m_map_node_records;
  std::set<std::string> m_vehicles_registered_for_option;
  std::map<std::string, std::string> m_vechicle_opt_status;
  std::map<std::string, double> m_bloom_sensor_reading; 

  std::set<std::string> m_vehicles_in_bloom;
  std::set<std::string> m_vehicles_in_storm;

  std::set<std::string> m_blooms_sampled_labels;
  std::set<std::string> m_blooms_detected_labels;
};

#endif 
