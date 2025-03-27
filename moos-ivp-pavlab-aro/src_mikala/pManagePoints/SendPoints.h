/************************************************************/
/*    NAME: Mikala Molina                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SendPoints.h                                          */
/*    DATE: June 1 2023                             */
/************************************************************/

#ifndef SendPoints_HEADER
#define SendPoints_HEADER
#include "XYFieldGenerator.h"
#include "XYPolygon.h"
#include "XYFormatUtilsPoly.h"
#include "VisitPoint.h"
#include "NodeReport.h"
#include "XYSegList.h"
#include "XYPoint.h"

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
using namespace std;

class SendPoints : public AppCastingMOOSApp
{
 public:
   SendPoints();
   ~SendPoints();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   XYPoint generateNewPoint(); 
   bool checkVisited(VisitPoint point, string vname);
   int  countUnvisited(string vname);
   
   

 private: // Configuration variables
 string m_region; 
 int m_initial_points;
 double m_min_dist; //default 10 
 vector<string> m_vehicle_names;
 int m_number_of_vehicles;
 int m_minimum_points; //default 1
 int m_capture_radius; 

 private: // State variables
 XYPolygon m_region_poly;
 XYFieldGenerator m_region_fld;
 string m_requesting_vehicle;
 bool m_need_to_send;
 map<string, bool> m_vehicle_awake;
 map<string, int> m_id_counter;
 map<string, vector<VisitPoint> > m_visit_points;
 map<string, vector<VisitPoint> > m_map_unvisited_points;
 map<string, bool> m_map_need_points; 
 map<string, XYSegList> m_visit_point_seglists; 
 map<string, vector<XYPoint> > m_nav_xy_points;
 map<string, bool> m_needs_initial_points;
 map<string, int> m_unvisited_count; 
 map<string, int> m_visited_count;
 map<string, vector<XYPoint> > m_visited_points;
 bool m_all_initial_points_sent;
 bool m_any_need_points;
 XYPoint m_found_point;
 
 std::string debug1;
  std::string debug2;
  std::string debug3;
 
};

#endif 
