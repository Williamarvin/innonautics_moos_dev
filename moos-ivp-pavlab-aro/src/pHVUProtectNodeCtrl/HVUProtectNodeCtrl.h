/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: HVUProtectNodeCtrl.h                            */
/*    DATE: May 2024                                        */
/************************************************************/

#ifndef HVUProtectNodeCtrl_HEADER
#define HVUProtectNodeCtrl_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "GeomUtils.h"         // for dist to point
#include "XYFormatUtilsPoly.h" // for string2poly
#include "XYPolygon.h"         // for region
#include "XYPoint.h"           // for XY centriod point
#include "NodeRecord.h"        // for fake node record
#include "NodeRecordUtils.h"   // for processing incoming node reports
#include "AngleUtils.h"        // for relAng
#include "MissionTask.h"       // for mission task assembly
#include <limits>              // for std::numeric_limits<double>::max();

class HVUProtectNodeCtrl : public AppCastingMOOSApp
{
 public:
   HVUProtectNodeCtrl();
   ~HVUProtectNodeCtrl();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool GeodesySetup();
   
 private:
   bool calculateInputs(double time);
   bool publishInputs();
   
   void updateIntruderList(); 
   void updateOptionsActive();
   void parseMissionTask(std::string spec, bool &stop, std::string &name, double &x, double &y);

   bool publishIgnoreLists();
   bool buildVoronoiActiveList(std::string msg);
   bool handleOptionMsg(std::string msg);
   bool handleNodeReport(std::string msg);
   void handleMissionTask(std::string msg);
   double updateClosestIntruder(); 
   
   bool postNewRegionInfo(); 
   

   
 private: // Configuration variables

   double m_region_update_thresh_dist;

   double m_intercept_input_range_of_max_val;
   double m_intercept_input_range_of_min_val;
   double m_intercept_input_max_val;
   double m_intercept_input_min_val;

 private: // State variables
   CMOOSGeodesy m_geodesy;
   double m_nav_x;
   double m_nav_y;

   std::vector<double> m_nav_x_vec;
   std::vector<double> m_nav_y_vec;

   double m_odometry;

   // Opinion Input values.
   // Key is the variable name
   std::map<std::string, double> m_opinion_inputs;

   // Contact and ignore set related
   std::string m_cur_option;

   bool m_send_new_ignore_set; 
   std::set<std::string> m_contacts;
   std::set<std::string> m_voronoi_active;

   // Region states
   std::string m_inside_zone_list;
   std::string m_outside_zone_list;
   XYPoint m_inside_zone_centriod;
   XYPoint m_outside_zone_centriod;
   std::string m_hvu_name; 

   // Battery input related
   double m_min_batt;
   double m_max_batt;
   bool m_got_real_batt_voltage; 
   double m_m300_batt_voltage;
   double m_current_batt_voltage;
   double m_time_last_got_batt;
   double m_current_batt_voltage_filtered; 

   // Region related
   XYPolygon m_inside_zone_poly;
   XYPolygon m_outside_zone_poly;
   XYPolygon m_stop_intercept_zone_poly;
   double m_stop_zone_buffer;
   double m_cumulative_rad_change; 

   double m_last_ignore_list_sent; 


   // dynamic updating of group trail related:
   std::set<std::string> m_intruder_names; 
   std::map<std::string, NodeRecord> m_node_rec_map;
   std::list<MissionTask> m_cur_mission_tasks;

   std::string m_closest_intruder_name; 

   
};

#endif 
