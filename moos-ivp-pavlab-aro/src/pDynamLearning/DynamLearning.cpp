/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DynamLearning.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "DynamLearning.h"

using namespace std;

//---------------------------------------------------------
// Constructor

DynamLearning::DynamLearning()
{
  // Set default values
  m_state_var_prefix = "GT";
  m_file_path = "/home/student2680/";
  m_last_save_time = MOOSTime();
  m_save_interval = 300;   // seconds
				   
  // Time Variables
  m_max_time_lim = 0.1;
  m_time_between_estimates = 0.4; //  This was typical for Herons
  m_num_readings_received = 0;
  m_num_est_made = 0;
  m_time_of_last_est = 0.0;
  
  // RNN values
  m_num_neurons = 20;
  m_num_inputs  = 5;

  m_scale_input_1 = 0;
  m_scale_input_2 = 0;
  m_scale_input_3 = 0;
  m_scale_input_4 = 0;

  m_alpha   = 0;
  m_beta1   = 0;
  m_beta2   = 0;
  m_epsilon = 0;
  m_contraction_error_weight = 0.1;

  // AID values
  m_num_of_params_aid = 9;
  
  m_aid_param_1_init = 0;
  m_aid_param_2_init = 0;
  m_aid_param_3_init = 0;
  m_aid_param_4_init = 0;
  m_aid_param_5_init = 0;
  m_aid_param_6_init = 0;
  m_aid_param_7_init = 0;
  m_aid_param_8_init = 0;
  m_aid_param_9_init = 0;
  
  m_param_1_gain = 0;
  m_param_2_gain = 0;
  m_param_3_gain = 0;
  m_param_4_gain = 0;
  m_param_5_gain = 0;
  m_param_6_gain = 0;
  m_param_7_gain = 0;
  m_param_8_gain = 0;
  m_param_9_gain = 0;
  m_a_m          = 0;

  // RLs values
  m_num_of_params_rls = 0;
  m_order_rls         = 0;
  m_forgetting_factor = 0;
  m_rls_param_1_init  = 0;
  m_rls_param_2_init  = 0;
  m_rls_param_3_init  = 0;
  m_rls_param_4_init  = 0;
  m_rls_param_5_init  = 0;
  m_rls_param_6_init  = 0;
  m_rls_param_7_init  = 0;
  m_rls_param_8_init  = 0;
  m_rls_param_9_init  = 0;

  // Vars to hold incoming message values
  m_speed_received   = false;
  m_heading_received = false;
  m_thrustR_received = false;
  m_thrustL_received = false;

  m_speed_received_time   = 0.0;
  m_heading_received_time = 0.0;
  m_thrustR_received_time = 0.0;
  m_thrustL_received_time = 0.0;

  m_nav_speed      = 0.0;
  m_nav_heading    = 0.0;
  m_pydir_thrust_R = 0.0;
  m_pydir_thrust_L = 0.0;

  //State Variables
  m_state_time_t = 0.0;
  m_state_time_t_minus_1 = 0.0;
  m_state_time_t_minus_2 = 0.0;

  m_state_t.clear();
  m_state_t_minus_1.clear();
  m_state_t_minus_2.clear();

  m_vel_est_rnn  = 0.0;
  m_vel_est_aid  = 0.0;
  m_vel_est_rls  = 0.0;

  m_nav_speed_est_ave  = 0.0;
  m_nav_speed_est_var  = 0.0;

  m_deploy = false;
  

}

//---------------------------------------------------------
// Destructor

DynamLearning::~DynamLearning()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool DynamLearning::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    double dval   = msg.GetDouble();
    double mtime = msg.GetTime();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
    
    if(key == ( m_state_var_prefix + "_SPEED") ) {
      m_speed_received = true;
      m_speed_received_time = mtime;
      m_nav_speed = dval;
    }
    else if(key == ( m_state_var_prefix + "_HEADING") ) {
      m_heading_received = true;
      m_heading_received_time = mtime;
      m_nav_heading = dval;
    }
    else if(key == "PYDIR_THRUST_R") {
      m_thrustR_received = true;
      m_thrustR_received_time = mtime;
      m_pydir_thrust_R = dval;
    }
    else if(key == "PYDIR_THRUST_L") {
      m_thrustL_received = true;
      m_thrustL_received_time = mtime;
      m_pydir_thrust_L = dval;
    }
    else if(key == "DEPLOY") {
      setBooleanOnString(m_deploy, msg.GetString());
    }

    
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool DynamLearning::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool DynamLearning::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Bail if not deployed
  if (not m_deploy ){
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // Step 0:  Compute an estimate of the forward speed using
  //          RNN, AID, and RLS methods.
  //          Only attempt the estimation every TimeBetweenEst
  //          seconds. A successful attempt requires fresh
  //          input data.

  // First check that to see if a message was recieved recently.
  // If not, then set all estimators to zero to be safe
  double heart_beat = 3.0; // seconds
  bool cond1 = ( (MOOSTime() - m_heading_received_time) >= heart_beat );
  bool cond2 = ( (MOOSTime() - m_thrustR_received_time) >= heart_beat );
  bool cond3 = ( (MOOSTime() - m_thrustL_received_time) >= heart_beat );
  if (cond1 and cond2 and cond3) {
    // Set to zero
    m_vel_est_rnn = 0.0;
    m_vel_est_aid = 0.0;
    m_vel_est_rls = 0.0;

    // Publish messages
    //this->publishMessages();
    // Update time
    m_time_of_last_est = MOOSTime();
  }
  bool cond4 = ( (MOOSTime() - m_time_of_last_est) >= m_time_between_estimates );
  bool cond5 = ( (MOOSTime() - m_heading_received_time) <= m_time_between_estimates );
  bool cond6 = ( (MOOSTime() - m_thrustR_received_time) <= m_time_between_estimates );
  bool cond7 = ( (MOOSTime() - m_thrustL_received_time) <= m_time_between_estimates );
  bool cond8 = ( m_num_readings_received >= 2 );

  if (cond4 and cond5 and cond6 and cond7 and cond8) {  // This could be an else if...
    // Generate estimates of forward speed using each method.
    // RNN ///////////////
    std::vector<double> feedforward_inputs;
    feedforward_inputs.clear();
    feedforward_inputs.push_back(m_vel_est_rnn * m_scale_input_1);
    feedforward_inputs.push_back( 0.5 + 0.5 * sin(m_nav_heading * PI/180.0) );
    feedforward_inputs.push_back( 0.5 + 0.5 * cos(m_nav_heading * PI/180.0) );
    feedforward_inputs.push_back(m_pydir_thrust_L * m_scale_input_3);  
    feedforward_inputs.push_back(m_pydir_thrust_R * m_scale_input_4);  
    
    //m_vel_est_rnn = m_SimpleRNN_obj.forwardProp(feedforward_inputs) / m_scale_input_1;

    m_vel_est_rnn = m_SimpleRNN_obj.forwardProp(feedforward_inputs);
    
    // AID ///////////////
    double dt = MOOSTime() - m_time_of_last_est;  // guaranteed <= heart_beat

    // compute the heading rate and check that less than 45
    double heading_rate = 0.0;
    // The state t_minus_1 is only available after two readings
    if ( (dt > 0) and (m_num_readings_received >= 2) ) {
      heading_rate = (m_state_t[1] - m_state_t_minus_1[1] ) / dt;
    }
    if ( abs(heading_rate) > 45) {
      heading_rate = 0.0;   // assume error. 
    }
    
    std::vector<double> f_hat_est;
    f_hat_est.clear();
    f_hat_est.push_back( m_pydir_thrust_L);
    f_hat_est.push_back( m_pydir_thrust_L * m_pydir_thrust_L );
    f_hat_est.push_back( m_pydir_thrust_L * abs(m_vel_est_aid) );
    f_hat_est.push_back( m_pydir_thrust_R);
    f_hat_est.push_back( m_pydir_thrust_R * m_pydir_thrust_R );
    f_hat_est.push_back( m_pydir_thrust_R * abs(m_vel_est_aid) );
    f_hat_est.push_back( abs(m_vel_est_aid) * m_vel_est_aid );
    f_hat_est.push_back( m_vel_est_aid );
    f_hat_est.push_back( abs(heading_rate * m_vel_est_aid) );
    
    double v_dot = m_SimpleAID_obj.estVelDot(f_hat_est);

    m_vel_est_aid = m_vel_est_aid + dt * v_dot;
    // Clip it
    if ( m_vel_est_aid < 0.0 )
      m_vel_est_aid = 0.0;

    // RLS ////////////////
    arma::Col<double> rls_inputs_est = { m_pydir_thrust_L, pow(m_pydir_thrust_L,2), pow(m_pydir_thrust_L,3),
					 m_pydir_thrust_R, pow(m_pydir_thrust_R,2), pow(m_pydir_thrust_R,3),
					 sin(m_nav_heading * PI/180), cos(m_nav_heading * PI/180), abs(heading_rate)};
    arma::Col<double> rls_params = m_rls_model_speed->get_weights();
    m_vel_est_rls = arma::dot(rls_inputs_est, rls_params);

    // Publish messages
    this->publishMessages();
    // Update time
    m_time_of_last_est = MOOSTime();

    m_num_est_made++;
  }

  AppCastingMOOSApp::PostReport();

  // On-line learning is in steps 1-3

  // Step 1:  Collect and orgainize asyncronous data for
  //          learning.  This involved checking that all
  //          data has been recieved - the bool flags, and
  //          that that time span from the first message
  //          to the most recent message in this group are
  //          within a time span.   If so, then collect all
  //          the states in a single vector for use in the
  //          learning algorithms.
  //          If only the inputs such as thruster commands,
  //          heading, etc. are present, then save tho
  
  double max_time = std::max({m_speed_received_time, m_heading_received_time, m_thrustR_received_time, m_thrustL_received_time});
  double min_time = std::min({m_speed_received_time, m_heading_received_time, m_thrustR_received_time, m_thrustL_received_time});
  double time_span = max_time - min_time;

  bool all_recieved = m_speed_received and m_heading_received and m_thrustR_received and m_thrustL_received;


  
  // Check if everything was recieved within the time window for learning 
  if ( all_recieved and ( time_span <= m_max_time_lim) ) {

    // First move back the previous values.  These will be empty the first
    // time arround.
    m_state_t_minus_2 = m_state_t_minus_1;
    m_state_time_t_minus_2 = m_state_time_t_minus_1;
    m_state_t_minus_1 = m_state_t;
    m_state_time_t_minus_1 = m_state_time_t;

    // save these new entries as a single vector
    // NAV_SPEED (m/s) | NAV_HEADING (deg) | PYDIR_THRUST_L | PYDIR_THRUST_R
    
    m_state_t.clear();
  
    m_state_t.push_back(m_nav_speed);
    m_state_t.push_back(m_nav_heading);
    m_state_t.push_back(m_pydir_thrust_L);
    m_state_t.push_back(m_pydir_thrust_R);
    // Save the timestamp of the last reading to be reported
    m_state_time_t = max_time;

    // Clear the message states for completeness
    m_nav_speed      = 0.0;
    m_nav_heading    = 0.0;
    m_pydir_thrust_R = 0.0;
    m_pydir_thrust_L = 0.0;
    m_speed_received_time   = 0.0;
    m_heading_received_time = 0.0;
    m_thrustR_received_time = 0.0;
    m_thrustL_received_time = 0.0;
    m_speed_received   = false;
    m_heading_received = false;
    m_thrustR_received = false;
    m_thrustL_received = false;

    m_num_readings_received++;
      
    // Step 2:  Check if more than one data set has been collected
    //          If two sets were recieved then set the initial value
    //          for the AID to be the most recent state.

    if (m_num_readings_received == 2){  
      // Initialize the AID
      m_SimpleAID_obj.setInitialvHat( m_state_t[0]);
    }
    
    // Step 3:  Update the models/networks with errors and save
    //          the state for the next time.
    //          AKA - learning and adaptation

    if ( (m_num_readings_received >= 3) and this->checkDataSet() ) {
      bool updateOK = this->updateEstimators();
    }

    // Finally update the estimator states to be the most recent
    // value measured.  This does not effect the learning code since
    // that information is stored in the state vectors for time t,
    // t-1, and t-2.  By doing this we insure that if NAV_SPEED is
    // intermittent, then the estimators will use the most recent
    // measurement to begin their estimation going forward. 
    m_vel_est_rnn = m_state_t[0] * m_scale_input_1;
    m_vel_est_aid = m_state_t[0];
    m_vel_est_rls = m_state_t[0];

  } //  End of check for a set of data to use in learning

  //  Finally, save data if enough time has passed
  if  ((MOOSTime() - m_last_save_time) > m_save_interval ) {
    this->saveParams();
    m_last_save_time = MOOSTime();
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool DynamLearning::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;

    if(param == "statevarprefix") {
      handled = setNonWhiteVarOnString(m_state_var_prefix, value);
    } else if(param == "savefilepath") {
      handled = setNonWhiteVarOnString(m_file_path, value);
    // Max Time Limit for State
    } else if(param == "maxtimelim") {
      handled = setPosDoubleOnString(m_max_time_lim, value) ;
    } else if(param == "timebetweenest") {
      handled = setPosDoubleOnString(m_time_between_estimates, value) ;
      
    // Recurrent Neural Network parameters ////////////////
    } else if(param == "numberneurons") {
      m_num_neurons = std::stoull(stripBlankEnds(value)) ;
      handled = isNumber(value);
    }
    else if(param == "numberinputs") {
      m_num_inputs = std::stoull(stripBlankEnds(value));
      handled = isNumber(value);
    }
    else if(param == "maxinput1") {
      double max_input_1;
      handled = setPosDoubleOnString(max_input_1, value);
      if (handled)
	m_scale_input_1 = 1.0 / max_input_1;
    }
    else if(param == "maxinput2") {
      double max_input_2;
      handled = setPosDoubleOnString(max_input_2, value);
      if (handled)
	m_scale_input_2 = 1.0 / max_input_2;
    }
    else if(param == "maxinput3") {
      double max_input_3;
      handled = setPosDoubleOnString(max_input_3, value);
      if (handled)
	m_scale_input_3 = 1.0 / max_input_3;
    }
    else if(param == "maxinput4") {
      double max_input_4;
      handled = setPosDoubleOnString(max_input_4, value);
      if (handled)
	m_scale_input_4 = 1.0 / max_input_4;
    }
    else if(param == "alpha") {
      handled = setPosDoubleOnString(m_alpha, value);
    }
    else if(param == "beta1") {
      handled = setPosDoubleOnString(m_beta1, value);  
    }
    else if(param == "beta2") {
      handled = setPosDoubleOnString(m_beta2, value);
    }
    else if(param == "epsilon") {
      handled = setPosDoubleOnString(m_epsilon, value);
    }
    else if(param == "contractionweight") {
      handled = setPosDoubleOnString(m_contraction_error_weight, value);
    
    // AID parameters ///////////////////////////////////
    } else if(param == "numberparamsaid") {
      m_num_of_params_aid = std::stoull(stripBlankEnds(value));
      handled = isNumber(value);  
    }
    else if(param == "aidparaminit1") {
      handled = setDoubleOnString(m_aid_param_1_init, value);
    }    
    else if(param == "aidparaminit2") {
      handled = setDoubleOnString(m_aid_param_2_init, value);
    } 
    else if(param == "aidparaminit3") {
      handled = setDoubleOnString(m_aid_param_3_init, value);
    }    
    else if(param == "aidparaminit4") {
      handled = setDoubleOnString(m_aid_param_4_init, value); 
    }
    else if(param == "aidparaminit5") {
      handled = setDoubleOnString(m_aid_param_5_init, value);
    }    
    else if(param == "aidparaminit6") {
      handled = setDoubleOnString(m_aid_param_6_init, value);
    } 
    else if(param == "aidparaminit7") {
      handled = setDoubleOnString(m_aid_param_7_init, value);
    }    
    else if(param == "aidparaminit8") {
      handled = setDoubleOnString(m_aid_param_8_init, value); 
    }
    else if(param == "aidparaminit9") {
      handled = setDoubleOnString(m_aid_param_9_init, value); 
    } 
    else if(param == "gainparam1") {
      handled = setDoubleOnString(m_param_1_gain, value);
    }    
    else if(param == "gainparam2") {
      handled = setDoubleOnString(m_param_2_gain, value);
    } 
    else if(param == "gainparam3") {
      handled = setDoubleOnString(m_param_3_gain, value);  
    }    
    else if(param == "gainparam4") {
      handled = setDoubleOnString(m_param_4_gain, value); 
    }
    else if(param == "gainparam5") {
      handled = setDoubleOnString(m_param_5_gain, value);
    }    
    else if(param == "gainparam6") {
      handled = setDoubleOnString(m_param_6_gain, value);
    } 
    else if(param == "gainparam7") {
      handled = setDoubleOnString(m_param_7_gain, value);  
    }    
    else if(param == "gainparam8") {
      handled = setDoubleOnString(m_param_8_gain, value); 
    }
    else if(param == "gainparam9") {
      handled = setDoubleOnString(m_param_9_gain, value); 
    } 
    else if(param == "am") {
      handled = setDoubleOnString(m_a_m, value);
      
    //  RLS Parameters ////////////////////////////
    } else if(param == "numberparamsrls") {
      m_num_of_params_rls = std::stoul(stripBlankEnds(value));
      handled = isNumber(value);  
    }
    else if(param == "orderrls") {
      m_num_of_params_rls = std::stoul(stripBlankEnds(value));
      handled = isNumber(value);  
    } 
    else if(param == "forgettingfactor") {
      handled = setDoubleOnString(m_forgetting_factor, value); 
    }    
    else if(param == "rlsparaminit1") {
      handled = setDoubleOnString(m_rls_param_1_init, value);  
    } 
    else if(param == "rlsparaminit2") {
     handled = setDoubleOnString(m_rls_param_2_init, value); 
    }
    else if(param == "rlsparaminit3") {
     handled = setDoubleOnString(m_rls_param_3_init, value); 
    }
    else if(param == "rlsparaminit4") {
     handled = setDoubleOnString(m_rls_param_4_init, value); 
    }
    else if(param == "rlsparaminit5") {
     handled = setDoubleOnString(m_rls_param_5_init, value); 
    }
    else if(param == "rlsparaminit6") {
     handled = setDoubleOnString(m_rls_param_6_init, value); 
    }
    else if(param == "rlsparaminit7") {
     handled = setDoubleOnString(m_rls_param_7_init, value); 
    }
    else if(param == "rlsparaminit8") {
     handled = setDoubleOnString(m_rls_param_8_init, value); 
    }
    else if(param == "rlsparaminit9") {
     handled = setDoubleOnString(m_rls_param_9_init, value); 
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  //  Initialize the AID, RNN, and RLS
  this->initializeEstimators();
  
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void DynamLearning::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register( ( m_state_var_prefix + "_SPEED"), 0);
  Register( ( m_state_var_prefix + "_HEADING") ,0);
  Register("PYDIR_THRUST_L", 0);
  Register("PYDIR_THRUST_R", 0);
  Register("DEPLOY", 0);
}


// ----------------------------------------------------------
//  Procedure:  Initialize all estimation methods
void DynamLearning::initializeEstimators()
{
  
  // Neural network setup /////////
  bool ok1, ok2, ok2_1, ok2_2, ok3;
  ok1 = m_SimpleRNN_obj.initialize(m_num_neurons, m_num_inputs);

  // Set the learning rates
  ok2   = m_SimpleRNN_obj.setLearningRateAdam(m_alpha, m_beta1, m_beta2, m_epsilon);
  ok2_1 = m_SimpleRNN_obj.setLearningRateContraction(m_contraction_error_weight);
  ok2_2 = m_SimpleRNN_obj.setUseContraction(true);

  // Set initial weights to be values calculated offline in Matlab
  // pretraining.  If unable to load from file, then use the values
  // entered manually. 

  Mat<double> pretrained_W(m_num_neurons, m_num_inputs, arma::fill::zeros);
  Col<double> pretrained_B(m_num_neurons,  arma::fill::zeros);
  Col<double> pretrained_A(m_num_neurons,  arma::fill::zeros);

  bool loadedW = pretrained_W.load( ( m_file_path + "W.csv" ), csv_ascii );
  bool loadedB = pretrained_B.load( ( m_file_path + "B.csv" ), csv_ascii );
  bool loadedA = pretrained_A.load( ( m_file_path + "A.csv" ), csv_ascii );

  if ( !(loadedW && loadedB && loadedA) ) {
    // Loading failed for at least one matrix.
    reportRunWarning("Warning: Failed to load latest RNN params, using original weights" );
    // Reset all matrix to the Matlab estimate

    // Need to reset these here becuase if the load fails, the size is
    // set to 0
    pretrained_W.zeros(m_num_neurons, m_num_inputs);
    pretrained_B.zeros(m_num_neurons);
    pretrained_A.zeros(m_num_neurons);

    // W Matrix
    pretrained_W(0,0)  =  0.0020;
    pretrained_W(1,0)  =  0.1283;
    pretrained_W(2,0)  =  0.0023;
    pretrained_W(3,0)  =  0.3888;
    pretrained_W(4,0)  =  0.0027;
    pretrained_W(5,0)  =  0.0085;
    pretrained_W(6,0)  =  0.3170;
    pretrained_W(7,0)  =  0.0014;
    pretrained_W(8,0)  =  0.3685;
    pretrained_W(9,0)  =  0.0023;
    pretrained_W(10,0)  =  0.0118;
    pretrained_W(11,0)  =  0.1897;
    pretrained_W(12,0)  = -0.0884;
    pretrained_W(13,0)  =  0.0381;
    pretrained_W(14,0)  = -0.0044;
    pretrained_W(15,0)  =  0.2216;
    pretrained_W(16,0)  =  0.0036;
    pretrained_W(17,0)  =  0.0115;
    pretrained_W(18,0)  =  0.1149;
    pretrained_W(19,0)  =  0.0119;

    pretrained_W(0,1)  =  0.1971;
    pretrained_W(1,1)  =  0.9671;
    pretrained_W(2,1)  =  0.5336;
    pretrained_W(3,1)  =  0.7617;
    pretrained_W(4,1)  =  0.2738;
    pretrained_W(5,1)  =  0.0254;
    pretrained_W(6,1)  =  0.3112;
    pretrained_W(7,1)  =  0.4131;
    pretrained_W(8,1)  =  0.9559;
    pretrained_W(9,1)  =  0.4201;
    pretrained_W(10,1)  = -0.2747;
    pretrained_W(11,1)  =  0.8675;
    pretrained_W(12,1)  =  0.6296;
    pretrained_W(13,1)  =  0.4385;
    pretrained_W(14,1)  =  0.3812;
    pretrained_W(15,1)  =  0.5135;
    pretrained_W(16,1)  =  0.0239;
    pretrained_W(17,1)  = -0.0547;
    pretrained_W(18,1)  =  0.2604;
    pretrained_W(19,1)  =  0.1548;
    
    pretrained_W(0,2)  =  0.2343;
    pretrained_W(1,2)  =  0.8453;
    pretrained_W(2,2)  =  0.0835;
    pretrained_W(3,2)  =  0.5612;
    pretrained_W(4,2)  =  0.2032;
    pretrained_W(5,2)  =  0.3456;
    pretrained_W(6,2)  =  0.8311;
    pretrained_W(7,2)  =  0.4104;
    pretrained_W(8,2)  =  0.4834;
    pretrained_W(9,2)  =  0.5198;
    pretrained_W(10,2)  = -0.1704;
    pretrained_W(11,2)  =  0.2640;
    pretrained_W(12,2)  =  0.2864;
    pretrained_W(13,2)  =  0.1764;
    pretrained_W(14,2)  =  0.5995;
    pretrained_W(15,2)  = -0.0105;
    pretrained_W(16,2)  = -0.3007;
    pretrained_W(17,2)  =  0.2278;
    pretrained_W(18,2)  = -0.0026;
    pretrained_W(19,2)  = -0.0881;

    pretrained_W(0,3)  =  0.5374;
    pretrained_W(1,3)  =  0.9315;
    pretrained_W(2,3)  =  0.8805;
    pretrained_W(3,3)  =  0.5281;
    pretrained_W(4,3)  =  0.9032;
    pretrained_W(5,3)  =  0.4599;
    pretrained_W(6,3)  =  0.2575;
    pretrained_W(7,3)  =  0.2370;
    pretrained_W(8,3)  =  0.3070;
    pretrained_W(9,3)  =  0.3981;
    pretrained_W(10,3)  = -0.0153;
    pretrained_W(11,3)  =  0.8870;
    pretrained_W(12,3)  =  0.0632;
    pretrained_W(13,3)  =  0.2895;
    pretrained_W(14,3)  =  0.8916;
    pretrained_W(15,3)  =  0.3011;
    pretrained_W(16,3)  =  0.5198;
    pretrained_W(17,3)  =  0.7250;
    pretrained_W(18,3)  =  0.7987;
    pretrained_W(19,3)  =  0.8390;

    pretrained_W(0,4)  =  0.2147;
    pretrained_W(1,4)  =  0.2133;
    pretrained_W(2,4)  =  0.9218;
    pretrained_W(3,4)  =  0.4323;
    pretrained_W(4,4)  =  0.5278;
    pretrained_W(5,4)  =  0.6369;
    pretrained_W(6,4)  =  0.4941;
    pretrained_W(7,4)  =  0.5559;
    pretrained_W(8,4)  = -0.0381;
    pretrained_W(9,4)  =  0.4779;
    pretrained_W(10,4)  =  0.4545;
    pretrained_W(11,4)  =  0.8465;
    pretrained_W(12,4)  =  0.5005;
    pretrained_W(13,4)  =  0.4298;
    pretrained_W(14,4)  =  0.6676;
    pretrained_W(15,4)  =  0.3711;
    pretrained_W(16,4)  =  0.5409;
    pretrained_W(17,4)  =  0.5807;
    pretrained_W(18,4)  =  0.1824;
    pretrained_W(19,4)  =  0.5952;
    

    // B Matrix
    pretrained_B(0) =  0.4331;
    pretrained_B(1) =  0.6571;
    pretrained_B(2) =  0.5746;
    pretrained_B(3) =  0.8373;
    pretrained_B(4) =  0.0984;
    pretrained_B(5) =  0.1927;
    pretrained_B(6) =  0.2213;
    pretrained_B(7) =  0.5140;
    pretrained_B(8) =  0.7523;
    pretrained_B(9) =  0.1686;
    pretrained_B(10) =  0.4395;
    pretrained_B(11) =  1.0952;
    pretrained_B(12) =  0.9335;
    pretrained_B(13) =  0.6624;
    pretrained_B(14) =  0.3320;
    pretrained_B(15) =  0.1286;
    pretrained_B(16) =  0.3293;
    pretrained_B(17) =  0.1978;
    pretrained_B(18) =  0.0010;
    pretrained_B(19) =  0.0449;

    // A Matrix
    pretrained_A(0) =  0.0890;
    pretrained_A(1) = -0.0558;
    pretrained_A(2) =  0.0871;
    pretrained_A(3) = -0.0259;
    pretrained_A(4) =  0.0782;
    pretrained_A(5) =  0.0833;
    pretrained_A(6) = -0.0052;
    pretrained_A(7) =  0.0921;
    pretrained_A(8) = -0.0104;
    pretrained_A(9) =  0.0891;
    pretrained_A(10) =  0.0865;
    pretrained_A(11) = -0.0469;
    pretrained_A(12) = -0.0525;
    pretrained_A(13) = -0.0437;
    pretrained_A(14) = -0.0600;
    pretrained_A(15) =  0.0606;
    pretrained_A(16) =  0.0757;
    pretrained_A(17) =  0.0773;
    pretrained_A(18) =  0.0717;
    pretrained_A(19) =  0.0647;
  }

  ok3 = m_SimpleRNN_obj.setWeights(pretrained_W, pretrained_B, pretrained_A);
  
  // Check that the NN initialized ok
  if ( !(ok1 && ok2 && ok2_1 && ok2_2 && ok3) ) {
    reportRunWarning("Neural Network Initialization Error" );
  }
  

  // Adaptive Identifier //////////
  bool ok4, ok5, ok6, ok7;
  ok4 = m_SimpleAID_obj.initialize(m_num_of_params_aid);

  Col<double> pretrainedAIDParams(m_num_of_params_aid,  arma::fill::zeros);
  std::vector<double> pretrained_AID_params;

  
  if ( pretrainedAIDParams.load( ( m_file_path + "AIDParams.csv" ), csv_ascii ) ) {
  //if ( false ) {
    // Loading was successful.  
    pretrained_AID_params = conv_to< std::vector<double> >::from(pretrainedAIDParams);
  } else {
    reportRunWarning("Warning: Failed to load latest AID params, using original params" );
    pretrained_AID_params.push_back(m_aid_param_1_init);
    pretrained_AID_params.push_back(m_aid_param_2_init);
    pretrained_AID_params.push_back(m_aid_param_3_init);
    pretrained_AID_params.push_back(m_aid_param_4_init);
    pretrained_AID_params.push_back(m_aid_param_5_init);
    pretrained_AID_params.push_back(m_aid_param_6_init);
    pretrained_AID_params.push_back(m_aid_param_7_init);
    pretrained_AID_params.push_back(m_aid_param_8_init);
    pretrained_AID_params.push_back(m_aid_param_9_init);
  }

  ok5 = m_SimpleAID_obj.setParams(pretrained_AID_params);

  // Set the adaptation gains 
  std::vector<double> Gamma;
  Gamma.push_back(m_param_1_gain);
  Gamma.push_back(m_param_2_gain);
  Gamma.push_back(m_param_3_gain);
  Gamma.push_back(m_param_4_gain);
  Gamma.push_back(m_param_5_gain);
  Gamma.push_back(m_param_6_gain);
  Gamma.push_back(m_param_7_gain);
  Gamma.push_back(m_param_8_gain);
  Gamma.push_back(m_param_9_gain);
  
  ok6 = m_SimpleAID_obj.setAdaptationGains(Gamma, m_a_m);

  // Set the inital vel estimate to zero to start
  ok7 = m_SimpleAID_obj.setInitialvHat( 0.0 );
  
  // Check that the AID initialized ok
  if ( !(ok4 && ok5 && ok6 && ok7) ) {
      reportRunWarning("Adaptive Identifier Initialization Error" );
  }

  
  // RLS ////////////
  bool ok8;
  m_rls_model_speed = new identification::rls( m_num_of_params_rls, m_order_rls, m_forgetting_factor);
  
  // set the initial weights
  Col<double> pretrainedRLSParams(m_num_of_params_rls,  arma::fill::zeros);
  std::vector<double> pretrained_RLS_params;

  if ( pretrainedRLSParams.load( (m_file_path + "RLSParams.csv" ), csv_ascii ) ) {
  //if ( false ) {
    // Loading was successful
    pretrained_RLS_params = conv_to< std::vector<double> >::from(pretrainedRLSParams);
  } else {
    reportRunWarning("Warning: Failed to load latest RLS params, using original params" );
    pretrained_RLS_params.push_back(m_rls_param_1_init);
    pretrained_RLS_params.push_back(m_rls_param_2_init);
    pretrained_RLS_params.push_back(m_rls_param_3_init);
    pretrained_RLS_params.push_back(m_rls_param_4_init);
    pretrained_RLS_params.push_back(m_rls_param_5_init);
    pretrained_RLS_params.push_back(m_rls_param_6_init);
    pretrained_RLS_params.push_back(m_rls_param_7_init);
    pretrained_RLS_params.push_back(m_rls_param_8_init);
    pretrained_RLS_params.push_back(m_rls_param_9_init);
    
  }
  
  ok8 = m_rls_model_speed->set_weights(pretrained_RLS_params);
  
  // Check that the RLS initialized ok
  if ( !ok8 ) {
      reportRunWarning("Recursive Least Squares Estimator Initialization Error" );
  }

  
}

// ----------------------------------------------------------
//  Procedure: Check the dataset to see if it is reasonable
//             returns true if data seems good
bool DynamLearning::checkDataSet()
{

  if (m_state_t[0] > 1.0 / m_scale_input_1)
    return false;
  //if (m_state_t[1] > 1.0 / m_scale_input_2)  // heading check no longer required
  //  return false;
  if (m_state_t[2] > 1.0 / m_scale_input_3)
    return false;
  if (m_state_t[3] > 1.0 / m_scale_input_4)
    return false;
  if (m_state_t_minus_1[0] > 1.0 / m_scale_input_1)
    return false;
  //if (m_state_t_minus_1[1] > 1.0 / m_scale_input_2)
  //  return false;
  if (m_state_t_minus_1[2] > 1.0 / m_scale_input_3)
    return false;
  if (m_state_t_minus_1[3] > 1.0 / m_scale_input_4)
    return false;
  if (m_state_t_minus_2[0] > 1.0 / m_scale_input_1)
    return false;
  //if (m_state_t_minus_2[1] > 1.0 / m_scale_input_2)
  //  return false;
  if (m_state_t_minus_2[2] > 1.0 / m_scale_input_3)
    return false;
  if (m_state_t_minus_2[3] > 1.0 / m_scale_input_4)
    return false;

  // If we made it here then return true
  return true;
}

// ----------------------------------------------------------
//  Procedure: Update estimators.  This code is the core
//             "learning part" of this app
bool DynamLearning::updateEstimators()
{
  // Update the RNN ///////////////////////
  // Scale the inputs

  std::vector<double> RNN_inputs_t_minus_2;
  RNN_inputs_t_minus_2.clear();
  RNN_inputs_t_minus_2.push_back( m_state_t_minus_2[0] * m_scale_input_1);
  RNN_inputs_t_minus_2.push_back( 0.5 + 0.5 * sin( m_state_t_minus_2[1] * PI/180.0) );
  RNN_inputs_t_minus_2.push_back( 0.5 + 0.5 * cos( m_state_t_minus_2[1] * PI/180.0) );
  RNN_inputs_t_minus_2.push_back( m_state_t_minus_2[2] * m_scale_input_3);
  RNN_inputs_t_minus_2.push_back( m_state_t_minus_2[3] * m_scale_input_4);
  
  std::vector<double> RNN_inputs_t_minus_1;
  RNN_inputs_t_minus_1.clear();
  RNN_inputs_t_minus_1.push_back( m_state_t_minus_1[0] * m_scale_input_1);
  RNN_inputs_t_minus_1.push_back( 0.5 + 0.5 * sin( m_state_t_minus_1[1] * PI/180.0) );
  RNN_inputs_t_minus_1.push_back( 0.5 + 0.5 * cos( m_state_t_minus_1[1] * PI/180.0) );
  RNN_inputs_t_minus_1.push_back( m_state_t_minus_1[2] * m_scale_input_3); 
  RNN_inputs_t_minus_1.push_back( m_state_t_minus_1[3] * m_scale_input_4);
  

  // Get estimate of the speed for t minus 2
  double vel_estimate_t_minus_1 = m_SimpleRNN_obj.forwardProp(RNN_inputs_t_minus_2);

  // Calculate gradient of error (the error function is simply error^2)
  double error_t_minus_1 = vel_estimate_t_minus_1 - RNN_inputs_t_minus_1[0];

  // Recurrent part -> replace the measured velocity with the estimated velocity
  RNN_inputs_t_minus_1[0] = vel_estimate_t_minus_1 * m_scale_input_1;
  
  // Get estimate of the speed for t
  double vel_estimate_t = m_SimpleRNN_obj.forwardProp(RNN_inputs_t_minus_1);

  // Calculate the gradient of the error
  double error_t = vel_estimate_t - m_state_t[0] * m_scale_input_1;

  // Back propagate this recurrent error
  bool ok1 =  m_SimpleRNN_obj.backPropRecurrentAdam(error_t_minus_1, RNN_inputs_t_minus_2, error_t, RNN_inputs_t_minus_1);
  
  // Check that the NN updated ok
  if ( !ok1 ) {
      reportRunWarning("RNN Backprop Error" );
  }

  
  
  // Update AID //////////////////////////////////////////
  double dt = m_state_time_t - m_state_time_t_minus_1;

  // compute the heading rate and check that less than 45
  // The AID uses the heading rate from the last time step,
  // to update its weights. 
  double heading_rate_AID = 0.0;
  if (dt > 0) {
    heading_rate_AID = (m_state_t_minus_1[1] - m_state_t_minus_2[1] ) / dt;
  }
  if ( abs(heading_rate_AID) > 45) {
    heading_rate_AID = 0.0;   // assume error. 
  }

  
  // Define the state vector at t minus 1.  These were the
  // states that resulted in the speed at time t.
  std::vector<double> f_hat;
  f_hat.clear();
  f_hat.push_back( m_state_t_minus_1[2]);
  f_hat.push_back( m_state_t_minus_1[2] * m_state_t_minus_1[2]);
  f_hat.push_back( m_state_t_minus_1[2] * abs(m_state_t_minus_1[0]) );
  f_hat.push_back( m_state_t_minus_1[3]);
  f_hat.push_back( m_state_t_minus_1[3] * m_state_t_minus_1[3]);
  f_hat.push_back( m_state_t_minus_1[3] * abs(m_state_t_minus_1[0]) );
  f_hat.push_back( abs( m_state_t_minus_1[0] ) * m_state_t_minus_1[0] );
  f_hat.push_back( m_state_t_minus_1[0]);
  f_hat.push_back( abs( heading_rate_AID * m_state_t_minus_1[0] ) );



  // Update AID
  bool ok2 = m_SimpleAID_obj.updateParams(m_state_t[0], dt, f_hat);

    // Check that the AID updated ok
  if ( !ok2 ) {
      reportRunWarning("AID update Error" );
  }

  // Update RLS ////////////////////////////////

  // compute the heading rate and check that less than 45
  // The RLS uses the heading rate from the current time step,
  // to update it' weights. 
  double heading_rate_RLS = 0.0;
  if (dt > 0) {
    heading_rate_RLS = (m_state_t[1] - m_state_t_minus_1[1] ) / dt;
  }
  if ( abs(heading_rate_RLS) > 45) {
    heading_rate_RLS = 0.0;   // assume error. 
  }
  arma::Col<double> rls_inputs = { m_state_t[2], pow(m_state_t[2],2), pow(m_state_t[2],3),
				   m_state_t[3], pow(m_state_t[3],2), pow(m_state_t[3],3),
				   sin(m_state_t[1] * PI/180), cos(m_state_t[1] * PI/180), abs(heading_rate_RLS) };

  // Step the RLS
  m_rls_model_speed->step(rls_inputs, m_state_t[0]);

  return( ok1 and ok2);
}


// ----------------------------------------------------------
//  Procedure: Calculates and publishes messages
//             returns true if everything finishes
bool DynamLearning::publishMessages()
{
  // Publish messages
  double m_vel_est_rnn_out = m_vel_est_rnn / m_scale_input_1 ;
  Notify("NAV_SPEED_EST_RNN", m_vel_est_rnn_out);
  Notify("NAV_SPEED_EST_AID", m_vel_est_aid);
  Notify("NAV_SPEED_EST_RLS", m_vel_est_rls);
  
  // Calculate and publish averages and covariance
  m_nav_speed_est_ave =  ( m_vel_est_rnn_out + m_vel_est_aid + m_vel_est_rls) / 3.0 ;
  m_nav_speed_est_var = ( pow( m_vel_est_rnn_out - m_nav_speed_est_ave, 2)
			  + pow( m_vel_est_aid - m_nav_speed_est_ave, 2)
			  + pow( m_vel_est_rls - m_nav_speed_est_ave, 2) ) / (3.0);
  Notify("NAV_SPEED_EST_AVE", m_nav_speed_est_ave);
  Notify("NAV_SPEED_EST_VAR", m_nav_speed_est_var); 
  
  // If we made it here then return true
  return true;
}




//------------------------------------------------------------
// Procedure: buildReport()

bool DynamLearning::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: DynamLearning.cpp                     " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "                                            " << endl;
  m_msgs << "Velocity Estimates:                         " << endl;

  ACTable actab(5);
  actab << " RNN | AID  | RLS | Average | NAV_SPEED ";
  actab.addHeaderLines();
  double average = ( (m_vel_est_rnn / m_scale_input_1) + m_vel_est_aid + m_vel_est_rls ) / 3.0;
  actab << (m_vel_est_rnn / m_scale_input_1) << m_vel_est_aid << m_vel_est_rls << average << m_nav_speed;
  
  m_msgs << actab.getFormattedString() << endl;
  m_msgs << "Variance in Estimation = " << m_nav_speed_est_var << "             " << endl;  
  m_msgs << "                                            " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Number of Estimations Made = " << m_num_est_made << "             " << endl;
  m_msgs << "Number of Readings Received = " << m_num_readings_received << "             " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "RNN status:                                 " << endl;
  if ( m_SimpleRNN_obj.isRNNContracting() ) {
    m_msgs << "- RNN is contraction stable                  " << endl;
  } else {
    m_msgs << "- Warning!!  RNN is NOT contraction stable   " << endl;
  }
  m_msgs << "- Stability point(s): ";
  std::vector<double> stability_points = m_SimpleRNN_obj.getEqualibPoints();
  for (unsigned int i=0; i<stability_points.size(); i++) {
    m_msgs << doubleToString(stability_points[i]);
    if ( i < stability_points.size()-1) {
      m_msgs << ", ";
    }
  }
  m_msgs << endl;
  
  return(true);
}



//--------------------------------------------------------------
// Procedure: saveParams()
//            Returns true if successful. 

bool DynamLearning::saveParams() {

  // RNN params
  Mat<double> dataW(m_num_neurons, m_num_inputs, arma::fill::zeros);
  Col<double> dataB(m_num_neurons, arma::fill::zeros);
  Col<double> dataA(m_num_neurons, arma::fill::zeros);
  
  bool ok;
  ok = m_SimpleRNN_obj.getWeights(dataW, dataB, dataA);
  if (!ok) {
    reportRunWarning("Error getting matrix weights" );
    return(false);
  }
  //AID params
  vector<double>  dataAIDParams_vec = m_SimpleAID_obj.getParams();
  Col<double>  dataAIDParams = conv_to< Col<double> >::from(dataAIDParams_vec);
  
  // RLS params
  Col<double> dataRLSParams = m_rls_model_speed->get_weights();

  
  // Save each as the lastest weights. 
  bool ok2, ok3, ok4, ok5, ok6;
  ok2 = dataW.save( ( m_file_path + "W.csv" ), csv_ascii);
  ok3 = dataB.save( ( m_file_path + "B.csv" ), csv_ascii);
  ok4 = dataA.save( ( m_file_path + "A.csv" ), csv_ascii);
  ok5 = dataAIDParams.save( ( m_file_path + "AIDParams.csv" ), csv_ascii);
  ok6 = dataRLSParams.save( ( m_file_path + "RLSParams.csv" ), csv_ascii);
  
  if ( !(ok2 && ok3 && ok4 && ok5 && ok6) ) {
    reportRunWarning("Error saving matrix weights" );
    return(false);
  }
  
  // Record a copy at this time for history
  string time_stamp = doubleToStringX( MOOSTime(), 3);
  bool ok7, ok8, ok9, ok10, ok11;
  ok7  = dataW.save( ( m_file_path + time_stamp + "_W.csv" ), csv_ascii);
  ok8  = dataB.save( ( m_file_path + time_stamp + "_B.csv" ), csv_ascii);
  ok9  = dataA.save( ( m_file_path + time_stamp + "_A.csv" ), csv_ascii);
  ok10 = dataAIDParams.save( ( m_file_path + time_stamp + "_AIDParams.csv" ), csv_ascii);
  ok11 = dataRLSParams.save( ( m_file_path + time_stamp + "_RLSParams.csv" ), csv_ascii);

  if ( !(ok7 && ok8 && ok9 && ok10 && ok11) ) {
    reportRunWarning("Error saving matrix weights" );
    return(false);
  }


  return(true);
}

