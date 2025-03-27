/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PingSim.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef PingSim_HEADER
#define PingSim_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "EsriBathyGrid.h"
#include "XYFormatUtilsConvexGrid.h"
#include <ctime>         // For use of the time function
#include <unistd.h>      // For use of the getpid function
#include <random>        // For random

class PingSim : public AppCastingMOOSApp
{
 public:
   PingSim();
   ~PingSim();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool populateFromFile(int lat, int lon);
   void mirrorPoint(double p1x, double p1y, double &p2x, double &p2y);

 private: // Configuration variables
   std::string m_mode = "function";
   std::string m_filename = "";
   bool m_mirror_grid = false;
   EsriBathyGrid m_grid;
   EsriBathyGrid m_grid_op_region;
   double m_x1 = 0.0;
   double m_y1 = 0.0;  // a line from x1, y1 to x2, y2 define the 
   double m_x2 = 0.0;  // plane of symmetry for mirroring
   double m_y2 = 0.0;  
   bool m_mirror_set_up = false;
   
 private: // State variables
   double m_rand_dist = 0;
   double m_nav_x = 0;
   double m_nav_y = 0;
   double m_calc_depth = 0;
   double m_file_depth = 0;
   double m_noise_added = 0.0;

};

#endif 
