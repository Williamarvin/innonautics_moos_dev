/************************************************************/
/*    NAME: Filip Stromstad                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DynamicTrafficLight.h                           */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef DynamicTrafficLight_HEADER
#define DynamicTrafficLight_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include <string>
#include <map>

enum BlockStatus{
  UNBLOCKED,
  TEMP_BLOCKED,
  PHYSICALLY_BLOCKED,
};
//DEADLOCKED
//PERMANENTLY_BLOCKED


class DynamicTrafficLight : public AppCastingMOOSApp
{
 public:
   DynamicTrafficLight();
   ~DynamicTrafficLight();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables
    std::string m_name_os;
    double m_default_speed;
    double m_precision;

    double m_max_safety_dist;
    double m_min_safety_dist; 
    double m_max_horizon_meters;
    double m_min_horizon_meters;
    bool   m_use_dynamic_speed;
    bool   m_show_visualization;
    bool   m_synchronize;
    bool   m_turn_in_place;
    std::string m_mode;
    bool   m_given_control;

 private: // State variables
    std::map<std::string, XYSegList> m_trajectory_map;
    std::map<std::string, double> m_speed_map;
    std::map<std::string, std::map<std::string, BlockStatus>> m_block_matrix;
    std::map<std::string, std::map<std::string, double>> m_temp_block_dist_matrix;
    double m_temp_block_limit;
  
    
    std::vector<std::string> m_handshakes;
    double m_last_handshake_time = -1;
    std::vector<std::string> m_nodes_in_proximity;
    double m_proximity_range;    
    std::map<std::string, std::string> m_known_traj_numbers;
    int m_os_trajectory_number;

    double m_nav_x;
    double m_nav_y;
    double m_nav_heading;
    double m_compass_heading;
    bool   m_use_compass_heading;
    double m_nav_speed;

    double m_potential_permablock_timer;
    double m_potential_deadlock_timer;
    double m_deadlock_timer;

    double m_reverse_thrust;

};

#endif 

bool depthFirstSearch(std::string name, std::map<std::string, std::map<std::string, BlockStatus>> &block_matrix, std::vector<std::string> &visited, std::vector<std::string> &recursive_stack);
bool isPermablocked(std::string name, std::map<std::string, std::map<std::string, BlockStatus>> &block_matrix, std::map<std::string, XYSegList> &trajectory_map);