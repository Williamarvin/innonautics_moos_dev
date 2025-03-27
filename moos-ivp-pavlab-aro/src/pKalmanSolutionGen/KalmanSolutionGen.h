/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: KalmanSolutionGen.h                                          */
/*    DATE: May 26, 2020                                */
/************************************************************/

#ifndef KalmanSolutionGen_HEADER
#define KalmanSolutionGen_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class KalmanSolutionGen : public AppCastingMOOSApp
{
 public:
   KalmanSolutionGen();
   ~KalmanSolutionGen();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
 protected: // Application functions
   void handleReport(std::string report);
   void executeKalmanX();
   void executeKalmanY();
   void postEstimates();
   void initializeSolution();
   void initModel(double x, double y, double vx, double vy, double time);
 protected: //Apllication member variables
 //The current values for the Kalman Filter
   double m_x;
   double m_xo;
   double m_y;
   double m_yo;
   double m_vx;
   double m_vy;
   double m_time;
  //The Updated Values
   double m_x1;
   double m_y1;
   double m_vx1;
   double m_vy1;
   double m_time1;
   //Variables for initialization
   bool m_got_speed;
   double m_P[2][2];
   double m_Py[2][2];
   //Variable for Config Check
   bool m_defined;
   double m_px00;
   double m_px11;
   double m_py00;
   double m_py11;
   double m_meas_noise;
   double m_init_noise;
   bool m_got_report;
   double m_detections;
   bool m_got_solution;
   double m_solutions;
   //variables for solution comparison
   double m_heading;
   double m_speed;

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
};

#endif 
