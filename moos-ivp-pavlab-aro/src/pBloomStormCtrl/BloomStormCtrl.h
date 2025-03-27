/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BloomStormCtrl.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef BloomStormCtrl_HEADER
#define BloomStormCtrl_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "XYPoint.h"  // for bloom record
#include "XYFormatUtilsPoint.h" // for string2Point
#include "XYPolygon.h"  // for region check
#include "XYFormatUtilsPoly.h"  // for region check


#include "NodeMessage.h"  // In the lib_ufield library


class BloomStormCtrl : public AppCastingMOOSApp
{
 public:
   BloomStormCtrl();
   ~BloomStormCtrl();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool handleMailNodeReport(const std::string& node_report_str);

   bool handleOptionMsg(const std::string& msg);
   bool handleBloomMsg(const std::string& msg);
   
   void postDiscovery();
   void prepToDetermineSamplePoint();
   double getExtraCost(std::string agent_name);
   void updateSamplePoint();
  
   void clearSamplePoints();

   bool isTargetInfoFresh();
   bool isSearchValueInfoFresh();
   bool calculateInputs(double time);
   bool publishInputs();
   void updateOdometry();

   bool publishIgnoreLists();

 private: // Configuration variables

   std::string m_region_1_spec;
   std::string m_region_2_spec;
   XYPolygon m_region_1_poly;
   XYPolygon m_region_2_poly; 
   double m_sample_radius;
   double m_sample_time;
   double m_sample_capture_radius;
   bool m_fixed_coalitions; 


   // Control input calcs
   // All
   double m_all_input_gain;
   
   // Explore
   double m_no_sample_available_weight; // Still needed?
   double m_search_value_improvement_gain;
   double m_batt_exh_gain;
   double m_explore_bias; 

   // Exploit
   //double m_sampling_cost_improvement_gain;

   // Migrate
   double m_storm_detected_input_val;
   double m_finish_migration_val;
   double m_time_since_last_migration_thresh; 
   double m_time_since_last_migration_resistance_val;
   double m_time_since_no_samples_thresh;  // change the input after this point
   double m_time_since_no_samples_encouragement_slope;

 private: // State variables

   ////////////////////////////////////////////////////////////
   // General bookkeeping of the states of ownship and group

   //--- Ownship

   // own name, position and odometry related
   std::string m_vname; 
   double m_nav_x;
   double m_nav_y;
   double m_nav_hdg; 
   double m_odometry;
   std::vector<double> m_nav_x_vec;
   std::vector<double> m_nav_y_vec;
   
   // which of the options am I pursuing?
   std::string m_last_option; 
   std::string m_curr_option;  

   // Current opinion input values for each option
   // Key is the variable name
   std::map<std::string, double> m_opinion_inputs;

   // Battery voltage simulator (or actual voltage is used
   // if detected)
   double m_min_batt;
   double m_max_batt;
   bool m_got_real_batt_voltage; 
   double m_m300_batt_voltage;
   double m_current_batt_voltage;
   double m_battery_exhausted;
   double m_time_const_batt;
   double m_time_last_got_batt;
   double m_batt_input_gain; 


   //--- Group
   
   // Key for each map below is the vehicle name. 
   std::map<std::string, NodeRecord>   m_map_node_records;

   // Contact and ignore set related
   // I.e. sets contain names of agents who are pursuing that
   // option
   bool m_send_new_ignore_set; 
   std::set<std::string> m_contacts;        // all
   std::set<std::string> m_voronoi_active;  // exploring
   std::set<std::string> m_sample_active;   // exploiting


   ////////////////////////////////////////////////////////////
   // Related to option inputs


   // --- Explore (search) related

   // Bloom related
   bool m_bloom_detected;
   bool m_new_bloom_detected;
   unsigned int m_bloom_found_counter;
   bool m_no_waypoints_available_to_sample; 


   double m_value_with_participation;
   double m_value_with_participation_time;
   double m_value_without_participation;
   double m_value_without_participation_time;
   
   // --- Exploit (sample) related
   //bool m_sampling_now; 
   std::list<XYPoint> m_bloom_records;
   
   // Record of which agents are currently stopped and sampling
   // The second value is the time the sampling started
   std::map<std::string, double>  m_map_sampling_agents;

  //double m_dist_to_closest_sample;
   double m_currently_sampling_weight; 
   unsigned int m_number_sampled;

  
  double m_cost_with_participation;
  double m_cost_with_participation_time;
  double m_cost_without_participation;
  double m_cost_without_participation_time;

  std::string m_own_backup_target_spec; 
  std::string m_own_target;
  double m_own_target_time;
  double m_own_target_priority;
  double m_own_target_priority_time;
  

   
   // --- Migrate related
   bool m_next_region;         // 0=region1, 1=region2
   bool m_migration_done; 
   bool m_storm_detected;
   double m_time_of_last_migration_started; 
   double m_time_of_last_migration_completion;
   double m_time_of_last_sample_available;
   double m_time_of_sample_available; 




};

#endif 
