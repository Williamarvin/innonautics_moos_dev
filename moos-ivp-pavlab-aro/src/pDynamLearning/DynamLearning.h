/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DynamLearning.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef DynamLearning_HEADER
#define DynamLearning_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "SimpleNN.h"
#include "SimpleAID.h"
#include "SimpleRLS.h"  // For RLS
#include <armadillo>   // only needed to initialize the NN and write params
#include <cmath>       // for overloaded abs()


class DynamLearning : public AppCastingMOOSApp
{
 public:
   DynamLearning();
   ~DynamLearning();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void initializeEstimators();
   bool checkDataSet();
   bool updateEstimators();
   bool publishMessages();
   bool saveParams();

 private: // Configuration variables

   // MOOS State variable name
   std::string m_state_var_prefix;  // Ex. NAV or GPS or GT
   // Filepath to save the weights
   std::string m_file_path;
   double m_last_save_time;
   int m_save_interval;
   

   // Timing Variables
   double m_max_time_lim;
   double m_time_between_estimates;
   long long unsigned int m_num_readings_received;
   long long unsigned int m_num_est_made;
   double m_time_of_last_est;

   //  RNN variables
   long long unsigned int  m_num_neurons;
   long long unsigned int  m_num_inputs;

   // These values are used to scale the inputs to the range 0 to 1.
   // It is better to over estimate the max values than to underestimate
   // The units are the same as the inputs.
   double m_scale_input_1;
   double m_scale_input_2;  // no longer needed 
   double m_scale_input_3;
   double m_scale_input_4;

   // These values are used for the Adams back-propagation algorithm
   double m_alpha;
   double m_beta1;
   double m_beta2;
   double m_epsilon;
   // This value is used for the contraction loss function
   double m_contraction_error_weight;
   
   // AID variables
   long long unsigned int m_num_of_params_aid;

   // These hold the initial estimates and adaptation gains
   double m_aid_param_1_init;
   double m_aid_param_2_init;
   double m_aid_param_3_init;
   double m_aid_param_4_init;
   double m_aid_param_5_init;
   double m_aid_param_6_init;
   double m_aid_param_7_init;
   double m_aid_param_8_init;
   double m_aid_param_9_init;

   double m_param_1_gain;
   double m_param_2_gain;
   double m_param_3_gain;
   double m_param_4_gain;
   double m_param_5_gain;
   double m_param_6_gain;
   double m_param_7_gain;
   double m_param_8_gain;
   double m_param_9_gain;
   double m_a_m;   // gain for delta_v

   // RLS variables
   unsigned int m_num_of_params_rls;
   unsigned int m_order_rls;
   double m_forgetting_factor;
   double m_rls_param_1_init;
   double m_rls_param_2_init;
   double m_rls_param_3_init;
   double m_rls_param_4_init;
   double m_rls_param_5_init;
   double m_rls_param_6_init;
   double m_rls_param_7_init;
   double m_rls_param_8_init;
   double m_rls_param_9_init;
   

   // logging variables
   
   

 private: // State variables

   // Bools to track if messages have been received
   // Need a complete set to determine the state. 
   bool m_speed_received;
   bool m_heading_received;
   bool m_thrustR_received;
   bool m_thrustL_received;

   // To track time
   double m_speed_received_time;
   double m_heading_received_time;
   double m_thrustR_received_time;
   double m_thrustL_received_time;

   double m_nav_speed;
   double m_nav_heading;
   double m_pydir_thrust_R;
   double m_pydir_thrust_L;

   // objects for NN, AID, and RLS
   SimpleNN m_SimpleRNN_obj;
   SimpleAID m_SimpleAID_obj;
   identification::identificator* m_rls_model_speed;

   // State Variables
   double m_state_time_t;
   double m_state_time_t_minus_1;
   double m_state_time_t_minus_2;

   std::vector<double> m_state_t;
   std::vector<double> m_state_t_minus_1;
   std::vector<double> m_state_t_minus_2;

   double m_vel_est_rnn;
   double m_vel_est_aid;
   double m_vel_est_rls;

   double m_nav_speed_est_ave;
   double m_nav_speed_est_var;

   bool m_deploy;

   
};

#endif 
