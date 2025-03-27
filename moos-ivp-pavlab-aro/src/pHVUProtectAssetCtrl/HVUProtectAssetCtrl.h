/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: FldHVUProtectAssetCtrl.h                        */
/*    DATE: May 2024                                        */
/************************************************************/

#ifndef HVUProtectAssetCtrl_HEADER
#define HVUProtectAssetCtrl_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

#include "GeomUtils.h"         // for dist to point
#include "XYFormatUtilsPoly.h" // for string2poly
#include "XYPolygon.h"         // for region
#include "XYPoint.h"           // for XY centriod point
#include "NodeRecord.h"        // for fake node record
#include "NodeRecordUtils.h"   // for processing incoming node reports
#include "NodeMessage.h"       // for node messages
#include "MissionTask.h"       // for mission task assembly
#include "XYRangePulse.h"  // for send pulse

// for random noise in fake node report
#include <ctime>         // For use of the time function
#include <unistd.h>      // For use of the getpid function
#include <random>        // For random



class HVUProtectAssetCtrl : public AppCastingMOOSApp
{
 public:
   HVUProtectAssetCtrl();
   ~HVUProtectAssetCtrl();

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
   bool handleNodeReport(std::string str);
   bool postFakeHVUNodeReport();
   bool handleConfigIntruderNames(std::string str); 
   bool possiblyUpdateNVPZ(NodeRecord newNodeRecord);
   bool checkForIntruders();
   void taskNodesToIntercept();
   void taskNodesToStopIntercept(std::string intruder_name);
   void sendPulse(std::string color);
   
 private: // Configuration variables
   CMOOSGeodesy m_geodesy;
   double m_region_update_thresh_dist;

   double m_task_resend_interval;
   bool m_add_noise; 

 private: // State variables
   XYPolygon m_pursuit_trigger_poly;
   XYPoint m_pursuit_trigger_region_centriod;
   double m_cumulative_rad_change; 
   
   std::map<std::string, NodeRecord> m_intruder_node_rec_map;
   std::set<std::string> m_intruder_names;
   std::set<std::string> m_names_active_intruders;
   std::set<std::string> m_names_task_sent;

   unsigned int m_number_of_tasks_sent;
   double m_last_time_tasks_sent; 

   // Fake HVU node report related
   bool m_post_fake_hvu_node_report_to_all;
   bool m_post_fake_hvu_node_report;
   
   double m_last_fake_node_report_post_time;
   std::string m_hvu_name;

   

 
   
};

#endif 
