/************************************************************/
/*    NAME: Nick Gershfeld                                  */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BathyGrider.h                                   */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef BathyGrider_HEADER
#define BathyGrider_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "EsriBathyGrid.h"
#include "SimpleGPR.h"
#include "SimpleKalmanConsensus.h"
#include "XYCircle.h"

using namespace std;

class BathyGrider : public AppCastingMOOSApp
{
 public:
   BathyGrider();
   ~BathyGrider();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool handleConsensusSpec(string input, string &vname,
			    vector<double> &estimate,
			    vector<double> &variance,
			    double &deg_connectedness);

   void sendOutOwnEstimate();
   double dist(std::vector<double> v1, std::vector<double> v2);

   
 private: // Configuration variables

   // convert observations from mm to another unit
   // mm to meters default, feet is 0.00328
   double m_conversion_factor = 0.001; 

   double m_time_between_estimates = 0.5;

   // esri grid config params
   double m_origin_lat = 0;
   double m_origin_lon = 0;
   double m_no_data_value = 0;
   double m_delta_thresh = 0.01;         // only send delta info if greater than this
                                         // thresh.  0.1 = 10%

   ////////////////////////////////
   // GPR parameters
   double m_sensor_variance = 1;
   double m_kernel_length_scale = 0.005;
   double m_variance_threshold = 0.015;  // max variance allowed before considered "done"

   double m_gpr_time = 0;                // last gpr publish
   vector<vector<double>> m_omit_list;   // omit or limit samples in these grid spaces
   double m_omit_dist_thresh = 5.0;

   // GPR Timing - Option to reak up the estimation across several app-ticks         
   // one iteration at a time
   int m_iterations_to_do = 0;          
   int m_iterations_completed = 0;

   // only run gpr every few appticks
   int m_apptick_counter = 0;             // appticks since gpr
   int m_appticks_to_skip = 0;            // every N appticks run gpr

   /////////////////////////////////
   // Consensus parameters
   double m_kalman_process_noise = 1.0;
   unsigned int m_max_iterations = 3;
   double m_cons_timeout = 4;
   double m_cons_waittime = 2;
   double m_cons_period = 20;             // how often to perform consensus


 private: // State variables

   bool m_surveying = false;
   bool m_exploring = false;
   bool m_transiting = false;
  
   // grids
   EsriBathyGrid m_grid_gpr;
   EsriBathyGrid m_grid_cons;

   // names of variables stored in the grid
   vector<string> m_cell_vars;
   
   // observation times
   double m_new_time = 0;
   double m_x_time = 0;
   double m_y_time = 0;
   double m_dist_time = 0;
   double m_conf_time = 0;

   // observations
   double m_x = 0;
   double m_y = 0;
   double m_dist = 0;
   double m_conf = 1;

   // running count of observations
   int m_count = 0;

   // gpr vars
   SimpleGPR m_gpr = SimpleGPR(1,1, 0.005);    // gpr constructor

   // Vector of multivariate_states for the GPR
   std::vector< std::vector<double> > m_multivariate_states;

   // consensus
   SimpleKalmanConsensus m_consensus = SimpleKalmanConsensus(0,0);

   // Consensus management
   double m_cons_calc_timer = 0;
   double m_cons_time = 0;
   int m_total_own_submissions = 0;
   int m_total_other_submissions = 0;
   int m_total_requests_initiated = 0;
   int m_total_requests_recieved = 0;

   string m_vname = "";


};

#endif 
