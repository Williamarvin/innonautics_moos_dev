/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Option.h                                        */
/*    DATE: March 20th, 2023                                */
/************************************************************/


/* Class def for each option
Notes:  (1) If a social mode of cooperative or competitive is 
        specified then the weight for any inter agent opinion
        is the same. 

	The option class manages many of the important properties. 
        For example, if desired it will calculate the current 
        input for this option
*/


#include "Option.h"

//---------------------------------------------------------
// Constructor
Option::Option()
{
  m_name = "";
  m_resistance_weight = 10;
  m_social_mode = "";
  m_input_var = "";
  m_input_fun_type = "";
  m_input_fun_linear_spec = "";

  m_always_active = true;
  m_alpha         = 1.0;
  m_min_input_val = 0.0;
  m_max_input_val = 100.0;
  m_input_gain    = 1.0; 
  
  m_random_perturb_params = false;
  
}


//---------------------------------------------------------
// Destructor
Option::~Option()
{
  
}


//--------------------------------------------------------
// checkOptionOk:  Performs general checks to be sure
//                 everything was configured reasonably
bool Option::checkOptionOk(std::string& warn_msg)
{
  if(m_name == ""){
    warn_msg = "Option Error: No name found in config block";
    return(false);
  }
  if(m_input_var == ""){
    warn_msg = "Option Error: No input variable name found in config block";
    return(false);
  }
  if(m_input_fun_type == ""){
    warn_msg = "Option Error: No input function type found in config block";
    return(false);
  }
  if(m_social_mode == ""){
    warn_msg = "Option Error: No social mode found in config  block";
    return(false);
  }
  if(m_option_output_name.size() == 0){
    warn_msg = "Option Error: No output variable name found in config block";
    return(false);
  }

  //TODO configure social mode defaults.

  // Randomly perturb params if desired
  if (m_random_perturb_params){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0, 1.0);

    // alpha
    double rnd_val = distribution(generator); // rnd number with mean 0 and std_dev 1
    // at 5 sigma the params can change 25%
    m_alpha = m_alpha * (1.0 + 0.25/3.0*rnd_val);

    // beta
    std::map<std::string, double>::iterator it;
    for (it = m_beta.begin(); it != m_beta.end(); it++){
      rnd_val = distribution(generator);
      it->second = it->second * (1.0 + 0.25/3.0*rnd_val);
    }

    // gamma
    rnd_val = distribution(generator);
    m_gamma = m_gamma * (1.0 + 0.25/3.0*rnd_val);

    // delta
    for (it = m_delta.begin(); it != m_delta.end(); it++){
      rnd_val = distribution(generator);
      it->second = it->second * (1.0 + 0.25/3.0*rnd_val);
    }

  }
  
  return(true);
}


//---------------------------------------------------------
// Procedure: needToRegister
bool Option::needToRegister(std::string& var)
{
  var = m_input_var; 
  return(true);
}


//---------------------------------------------------------
// Procedure: setOptionOutput
bool Option::setOptionOutput(std::string var, std::string val)
{
  if ((var =="") || (val == "")) {
    return(false);
  }
  m_option_output_name.push_back( toupper(var) );
  m_option_output_val.push_back( val );
    
  return(true);
}


//---------------------------------------------------------
// Procedure: setSocialMode
bool Option::setSocialMode(std::string val)
{
  std::set<std::string> possible_modes;
  possible_modes.insert("custom");
  possible_modes.insert("cooperative");
  possible_modes.insert("competitive");
  possible_modes.insert("oblivious");

  if(possible_modes.count(val) <1)
    return(false);

  m_social_mode = val;
  
  return(true);
}

//---------------------------------------------------------
// Procedure: setIntraAgentInterOptionCoupling
bool Option::setIntraAgentInterOptionCoupling(std::string val)
{
  std::string temp_val = val;   // Not sure this is needed
  std::string other_option_name = biteStringX(temp_val,'=');
 
  if ((other_option_name == "") || (temp_val ==""))
    return(false);

  double temp_dbl; 
  bool ok = setDoubleOnString(temp_dbl, temp_val);
  if (ok){
    m_beta[other_option_name] = temp_dbl;
    //m_beta_option_names.push_back(other_option_name);
    //m_beta_option_vals.push_back(temp_dbl);
  }
  
  return(ok);
}



//---------------------------------------------------------
// Procedure: setInterAgentInterOptionCoupling
bool Option::setInterAgentInterOptionCoupling(std::string val)
{
  std::string temp_val = val;   // Not sure this is needed
  std::string other_option_name = biteStringX(temp_val,'=');
 
  if ((other_option_name == "") || (temp_val ==""))
    return(false);

  double temp_dbl; 
  bool ok = setDoubleOnString(temp_dbl, temp_val);
  if (ok){
    m_delta[other_option_name] = temp_dbl;
    //m_delta_option_names.push_back(other_option_name);
    //m_delta_option_vals.push_back(temp_dbl);
  }
  
  return(ok);
}

//-----------------------------------------------------------
// Procedure: setInputFunType;
bool Option::setInputFunType(std::string val)
{

  std::string temp_val = val;   // Not sure this is needed
  std::string function_name = biteStringX(temp_val,':');
  
  if (function_name == "none"){
    m_input_fun_type = "none";
    return(true);
    
    /*  } else if (function_name == "linear") {
    bool ok = handleLinFunctionString(temp_va);
    if (ok) {	
      m_input_fun_type = "linear";
      return(true);
    } else {
      return(false);
        } */
  } else if (function_name == "quadratic") {
    m_input_fun_type = "quadratic";
    return(true);
  } else {
    return(false); 
  }
}


//---------------------------------------------------------
// Procedure: setActiveCondition
bool Option::setActiveCondition(std::string var, std::string val)
{
  if ((var =="") || (val == "")) {
    return(false);
  }

  if (var == "always") {
    m_always_active = true;
    return(true);
  }
  
  m_active_condition_name.push_back( toupper(var) );
  m_active_condition_val.push_back( val );
  m_always_active = false;
    
  return(true);
}


//-----------------------------------------------------------
// Procedure: getConditionVar - with error checking. 
std::string Option::getConditionVar(unsigned int indx)
{
  if (indx >= m_active_condition_name.size())
    return("");

  return(m_active_condition_name[indx]); 
}

//-----------------------------------------------------------
// Procedure: getConditionVal - with error checking. 
std::string Option::getConditionVal(unsigned int indx)
{
  if (indx >= m_active_condition_val.size())
    return("");

  return(m_active_condition_val[indx]); 
}

//----------------------------------------------------------
// Proedure: computeInputB
double Option::computeInputB(double raw_input_val)
{
  
  double in; 
  // Clip
  if (raw_input_val > m_max_input_val) {
    in = m_max_input_val; 
  } else if ( raw_input_val < m_min_input_val) {
    in = m_min_input_val; 
  } else {
    in = raw_input_val; 
  }

  // scale to [-1, 1]
  double in_scaled = 2.0 * (in - m_min_input_val) / (m_max_input_val - m_min_input_val) - 1.0; 

  // Now scale up by input gain
  if (m_input_fun_type == "none"){
    return(in_scaled * m_input_gain); 
  } else if (m_input_fun_type == "quadratic") {
    return(in_scaled * in_scaled * m_input_gain);
  }
  
  // backup
  return(0.0); 
}



//----------------------------------------------------
//  Get the options outputs
//              Returns:  the number of outputs
//                        vector of variable names (by ref)
//                        vector of variable values (by ref)
unsigned int Option::getOptionOutputs(std::vector<std::string>& names, std::vector<std::string>& vals)
{
  unsigned int ret = m_option_output_name.size();
  if (ret != m_option_output_val.size() ){
    // Something is wrong
    return(0);
  }
  
  names = m_option_output_name;
  vals  = m_option_output_val; 
  
  return(ret);
}




// Utility to convert from strings in config file to Options
// Expect a vector of strings taken from the config block:
//
// An example, each line in the config blocks are given as
// an individual index in the vector of strings passed as
// arguments.  The order does not matter.
// [0]   name = wide_loiter
// [1]   option_output = OPTION=WIDE_LOITER
// [2]   resistance_weight = 0.01
// [3]   social_mode = normal   // locks out and resets
// [4]   intra_agent_same_option_coupling = 0.2  // Self reinforcement  (alpha)
// [5]   intra_agent_inter_option_coupling = small_loiter = 0.1      //  (beta)
// [6]   inter_agent_same_option_coupling = 0.1  // influence of others (gamma)
// [7]   inter_agent_inter_option_coupling = small_loiter = 0.1      // (delta)
// [8]   input = DANGER_LEVEL
// [9]   input_function_type = none
// [10]  input_max  = 100
// [11]  input_min  = 0
// [12]  active_condition = DEPLOY=true    // de ault options is 'always'
//

Option buildOptionFromSpec(std::list<std::string> spec, bool& ok, std::string& warn_msg)
{
  Option new_optn;

  // error checking
  if (spec.size() < 1) {
    ok = false;
    warn_msg = "Option Parameter Error: Empty config block found for option";
    return(new_optn);
  }

  // set options
  std::list<std::string>::iterator p;
  for (p=spec.begin(); p!=spec.end(); p++) {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    // strip out the comments after // 
    std::string value = biteStringX(line, '/');

    bool handled = false;
    
    if ((param != "") && (value == "")){
      // no value
      handled = false;
      
    } else if (param == "name") {
      handled = new_optn.setName(value);

    } else if (param == "option_output") {
      std::string output_var = toupper(biteStringX(value, '='));
      handled = new_optn.setOptionOutput(output_var, value);
      
    } else if (param == "resistance_weight") {
      double db_val;
      bool ok1 = setDoubleOnString(db_val, value);
      handled = (ok1 && new_optn.setResistanceWeight(db_val));
      
    } else if (param == "social_mode") {
      handled = new_optn.setSocialMode(value);

    } else if (param == "intra_agent_same_option_coupling") {
      double db_val;
      bool ok1 = setDoubleOnString(db_val, value);
      handled = (ok1 || new_optn.setIntraAgentSameOptionCoupling(db_val));
      
    } else if (param == "intra_agent_inter_option_coupling") {
      handled = new_optn.setIntraAgentInterOptionCoupling(value);

    } else if (param == "inter_agent_same_option_coupling") {
      double db_val;
      bool ok1 = setDoubleOnString(db_val, value);
      handled = (ok1 && new_optn.setInterAgentSameOptionCoupling(db_val) );

    } else if (param == "inter_agent_inter_option_coupling") {
      handled = new_optn.setInterAgentInterOptionCoupling(value);
      
    } else if (param == "input") {
      handled = new_optn.setInputVar(value);
      
    } else if (param == "input_gain") {
      double db_val;
      bool ok1 = setPosDoubleOnString(db_val, value);
      handled = (ok1 && new_optn.setInputGain(db_val));

    } else if (param == "input_function_type") {
      handled = new_optn.setInputFunType(value);
      
    } else if (param == "input_max") {
      double db_val;
      bool ok1 = setDoubleOnString(db_val, value);
      handled = (ok1 && new_optn.setMaxInputVal(db_val));

    } else if (param == "input_min") {
      double db_val;
      bool ok1 = setDoubleOnString(db_val, value);
      handled = (ok1 && new_optn.setMinInputVal(db_val));

    } else if (param == "active_condition") {
      std::string cond_var = tolower(biteStringX(value, '='));
      handled = new_optn.setActiveCondition(cond_var, value);

    } else if (param == "randomly_perturb_params"){
      bool randomly_perturb = false;
      handled = setBooleanOnString(randomly_perturb, value);
      if (handled)
	handled = new_optn.setRandPerturb(randomly_perturb);
      
    } else {
      // unrecognized input, check if empty line,
      // if not empty then return error
      handled = ((param == "") && (value == ""));
    }

    if (!handled) {
      warn_msg = "Option Parameter Error: Bad line = " + orig;
      ok = false;
      return(new_optn);
    }
    
  } // end of for loop

  // Check this Option is ok and return the determination
  std::string check_msg;
  ok = new_optn.checkOptionOk(check_msg);
  if (!ok) {
    warn_msg = check_msg;
  }
  
  return(new_optn);
}



