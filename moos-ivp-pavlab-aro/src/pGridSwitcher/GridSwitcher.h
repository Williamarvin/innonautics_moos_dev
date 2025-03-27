/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GridSwitcher.h                                  */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef GridSwitcher_HEADER
#define GridSwitcher_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "EsriBathyGrid.h"
#include "XYFormatUtilsConvexGrid.h"

class GridSwitcher : public AppCastingMOOSApp
{
 public:
   GridSwitcher();
   ~GridSwitcher();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

   std::string replaceGridLabel(std::string grid_spec, std::string vname);
   std::string replaceDeltaGridLabel(std::string grid_spec, std::string vname);
   
   bool populateFromFile(double lat, double lon);
   bool recalcGtGrid(std::string grid_spec);
   bool handleConfigInputVars(std::string val);
   bool handleFullGridMsg(std::string sval);
   bool handleDeltaGridMsg(std::string sval);
   void mirrorPoint(double p1x, double p1y, double &p2x, double &p2y);

   
   
 private: // Configuration variables
   bool m_mirror_grid = false;
   bool m_sim_run = false;
   
   double m_x1 = 0.0;
   double m_y1 = 0.0;  // a line from x1, y1 to x2, y2 define the 
   double m_x2 = 0.0;  // plane of symmetry for mirroring
   double m_y2 = 0.0;  
   

 private: // State variables

   // flags for switching, and recalculating
   bool m_label_switched;
   bool m_want_variance;
   bool m_recalc_gt_grid;

   // maps to hold the grid specs to switch, and if we
   // need to update from deltas
   std::map<std::string, EsriBathyGrid>  m_grids_received;
   std::map<std::string, bool> m_need_to_update_full_grid;
   std::map<std::string, bool> m_need_to_update_grid_delta;
   
   std::string m_current_grid_label;
   std::string m_vname;
   
   EsriBathyGrid m_gt_grid;
   EsriBathyGrid m_gt_grid_from_file;

   double m_lat_origin = 0.0;
   double m_long_origin = 0.0;

   // possible grid names 
   std::set<std::string> m_input_vars;
   std::set<std::string> m_input_vars_delta;

   
   std::string m_output_var;
   std::string m_switching_var;
   std::string m_gt_grid_filename;

};

#endif 
