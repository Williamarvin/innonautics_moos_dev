/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionInput.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef OpinionInput_HEADER
#define OpinionInput_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYRangePulse.h"  // for send pulse
#include "GeomUtils.h"     // for dist to point
#include "OpinionRecord.h"

class OpinionInput : public AppCastingMOOSApp
{
 public:
   OpinionInput();
   ~OpinionInput();

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
   double calculateDangerInput(double time) {return(0.0);};
   bool calculateInputs(double time);
   bool publishInputs();

   bool publishIgnoreLists(); 
   

 private: // State variables

   double m_nav_x;
   double m_nav_y;

   std::vector<double> m_nav_x_vec;
   std::vector<double> m_nav_y_vec;

   double m_odometry; 
   
   XYRangePulse m_danger_pulse;
   double       m_danger_time;
   unsigned int m_danger_pulse_count;

   // Linear input from max at range = 0
   // to level of 0 at range min dist
   double m_danger_max;
   double m_danger_min;
   double m_danger_min_range;  // Linear  to zero

   // Opinion Input values.
   // Key is the variable name
   std::map<std::string, double> m_opinion_inputs;

   // Contact and ignore set related
   bool m_send_new_ignore_set; 
   std::set<std::string> m_contacts;
   std::set<std::string> m_voronoi_active;

   // Battery input related
   double m_min_batt;
   double m_max_batt;
   bool m_got_real_batt_voltage; 
   double m_m300_batt_voltage;
   double m_current_batt_voltage; 


};

#endif 
