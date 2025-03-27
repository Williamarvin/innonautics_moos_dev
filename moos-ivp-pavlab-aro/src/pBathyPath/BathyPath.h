/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BathyPath.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef BathyPath_HEADER
#define BathyPath_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "EsriBathyGrid.h"
#include "PathTree.h"
#include "SimpleMDP.h"
#include <cmath>   // erf function
#include <random>   // MVI reward calc
#include <queue>
#include <limits>




class BathyPath : public AppCastingMOOSApp
{
 public:
   BathyPath();
   ~BathyPath();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
  //std::string getGridLabel(std::string grid_spec);

   bool getNextCellLookahead(double curr_x, double curr_y, double curr_heading, double &new_x, double &new_y, int &new_id);
   bool buildPathTree(int iterations, double curr_x, double curr_y, double curr_heading, PathTree &path);

   std::string getBestPath(PathTree pathtree);


  
 private: // Configuration variables
   double m_no_data_value = 0;
   double m_lat_origin = 0;
   double m_lon_origin = 0;
   int m_tree_depth = 3;
   bool m_use_UCB_reward = true;      // false = use MVI

   bool m_lawnmower = false;
   int m_vnum = 2;                    // number of vehicles in the survey
   int m_vN = 0;                      // N coefficient for lawnmower calculations
   int m_degrees = 0;                 // grid rotation degrees
  
 private: // State variables
   EsriBathyGrid m_grid;
   int m_grid_deltas_rcvd = 0;
   int m_full_grid_rcvd = 0;

   double m_x = 0;
   double m_y = 0;
   double m_heading = 0;
   int m_new_id = -1;
  
   XYSegList m_points_out;
   bool m_surveying = false;
   bool m_path_found = false;
   bool m_first_grid_received = false;

   std::string m_print = "";

   std::string m_lawnstring = "";

   // Estimates of Utilities
   std::map<std::string, double> m_utilities;
   
   // variables for UCB reward
   double m_round_number = 1;
   double m_prob_scale_factor = 10000;  // needed because Fast GPR skews the variance
   
   // variables for MVI reward
   double m_robot_noise = 1;            // total guess for now. 
   
};

#endif 
