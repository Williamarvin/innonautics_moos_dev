/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionManagerEngine.h                          */
/*    DATE: March 20th, 2023                                */
/************************************************************/


/* This class defines a opinion dynamics engine largely based
   on the paper "A General Model of Opinion Dynamics with Tunable
   Sensitivity", which can be found here:
   https://www.researchgate.net/publication/344180671_A_General_Model_of_Opinion_Dynamics_with_Tunable_Sensitivity
   The variable names are kept the same for clarity.
   Other papers from the Leonard Lab at Princeton are referenced as
   well. 
*/


#include "OpinionManagerEngine.h"

//---------------------------------------------------------
// Constructor

OpManagerEngine::OpManagerEngine()
{


  m_u_a_upper_bound = 1.0;
  m_u_a_lower_bound = 1.0; 
  m_u_d_upper_bound = 1.0;
  m_u_d_lower_bound = 1.0;
  
  m_tau_u = 10.0;
  m_u_min = 0.5;
  m_u_max = 1.5; 
  m_u_th  = 0.1;
  m_Hill_order = 2.0;
  m_opinion_thresh = 0.1; 
  
  m_u_i = 0.5;
  
  
  m_setup_complete = false;
  m_last_iter_time = 0;
  m_social_tick = 1;
  m_opinion_iter_count = 0;
  m_stale_thresh = 2.0;
  m_reset_time_interval = 5.0;
  m_reset_time_cmd = 0.0; 

  m_vname = "832F";
  m_group = "leader";

  m_num_int_gain = 1.0; 
}


//---------------------------------------------------------
// Destructor

OpManagerEngine::~OpManagerEngine()
{
  
}


//---------------------------------------------------------
// handleOptConfig
//        This function reads the config file, adds all valid
//        options to the engine, and returns true if successful
bool OpManagerEngine::handleOptConfig(std::string filename, std::string& warn_msg)
{

  std::list<std::string> config_lines;
  bool found_config_start = false;

  
  // Step 1.  Open the file and read each line
  std::ifstream file(filename);
  if (file.is_open()) {
    std::string line;
    while (std::getline(file, line)) {

      // remove comments
      std::string stripped_line = biteStringX(line, '/');
      std::cout << " stripped_line = " << stripped_line << std::endl;

      if (stripped_line == "") {
	continue; 
      }

      // Process the line
      if (stripped_line == "social_option"){
	// check if another block was found and not properly handled.
	if (found_config_start) {
	  warn_msg = "Opinion Manager Error: Encountered a config block that was not closed";
	  file.close();
	  return(false);
	}
	std::cout << "  Setting found_config_start = true" << std::endl;
	found_config_start = true;
	
      } else if ( (stripped_line != "{") && (stripped_line != "}") && found_config_start) {
	// must be a valid line, add this line to the vector of strings to process
	config_lines.push_back(stripped_line);
	std::cout << "  Pushing_back line" << std::endl;
	
      } else if (( stripped_line == "{") && !found_config_start) {
	// error here, found an opening bracket, but no start
	std::cout << "  Error here, found an opening bracket, but no start"  << std::endl;
	warn_msg = "Opinion Manager Error: Encountered a config block that was not started properly";
	file.close();
	return(false);

      } else if (( stripped_line == "}") && !found_config_start) {
	// error here, found an closing bracket, but no start
	warn_msg = "Opinion Manager Error: Encountered a config block that was not started properly";
	file.close();
	std::cout << "  Error here encountered a config block that was not started properly" << std::endl;
	return(false);

      }	else if (( stripped_line == "}") && found_config_start) {
	// found a valid closing bracket, process this block
	std::cout << "  Found a valid closing bracket, processing this block" << std::endl;
	std::string temp_warn_msg = "";
	bool ok = false;
	Option new_option = buildOptionFromSpec(config_lines, ok, temp_warn_msg);

	std::cout << "   Processed option ok = " << std::to_string(ok) << std::endl; 
	if (ok)  {
	  m_options.push_back(new_option);
	  std::string new_option_name = new_option.getOptionName();
	  m_option_names_hash[new_option_name] = static_cast<unsigned int>( m_options.size() -1 ); // will always be last
	} else {
	  warn_msg = temp_warn_msg;
	  file.close();
	  std::cout << "   Warn message = " << temp_warn_msg << std::endl;
	  return(false);
	}
	
	// reset
	config_lines.clear();
	found_config_start = false;
	std::cout << "  Reset config" << std::endl;
      }
      // ignore the rest of the lines
      
    }  // end of while reading file
    
    file.close();
  } else {
    warn_msg = "Opinion Manager Error: Cannot open config file";
    return(false);
  }

  // Did we read at least one option?
  if (m_options.size() < 1) {
    warn_msg = "Opinion Manager Error: Opened file but did not find any config blocks";
    return(false);
  }

  // Run set up with a dummy time for safety
  
  this->setupOpinionManager(m_last_iter_time);
  
  return(true);
}



//----------------------------------------------------------------------
//   setupOpinionManager()
//        General setup
bool OpManagerEngine::setupOpinionManager(double time)
{

  if (m_options.size() < 1)
    return(false);

  // Initialize opinions to be nuetral
  resetOpinionState(time, 0.0);
  
  // Initialize opinion_inputs to be zero
  for (int i = 0; i < m_options.size(); i++) {
    m_opinion_inputs.push_back(0.0); 
  }

  // Build vector of variable names to register
  std::vector<Option>::iterator it;
  std::vector<std::string> setup_vars_to_register; 
  for (it = m_options.begin(); it != m_options.end(); it++){
    
    // get the condition variables for this option
    unsigned int number_conditions = it->numberOfConditions();
    for (unsigned int j = 0; j<number_conditions; j++) {
      setup_vars_to_register.push_back(it->getConditionVar(j));
    }
    
    // get the input variable for this option
    setup_vars_to_register.push_back(it->getInputVar());

  }

  m_vars_to_register = setup_vars_to_register;
  
  m_last_iter_time = time;
  
  return(true);
}


// ----------------------------------------------------
//   resetOpinionState()
//   clears the opinions and sets all to zero.
//   Also resets the attention to the miniumum. 
//   prevents decision-making until the reset time has passed


bool OpManagerEngine::resetOpinionState(double time_now,
					double reset_time_interval)
{
  m_opinions.clear();
  for (int i = 0; i < m_options.size(); i++) {
    m_opinions.push_back(0.0); 
  }

  m_u_i = m_u_min;
  
  // Clear out any records we have.  This is done to
  // completely reset all local messages
  m_neighbor_opinion_records.clear();

  m_reset_time_cmd = time_now;
  m_reset_time_interval = reset_time_interval;   

  return(true); 
}

//-----------------------------------------------------------------------
// Check if a var name is in the list of vars to register
bool OpManagerEngine::isValInBuffer(std::string var)
{
  std::vector<std::string>::iterator p;
  for (p = m_vars_to_register.begin(); p != m_vars_to_register.end(); p++) {
    if (*p == var)
      return(true);
  }

  return(false);
}



//---------------------------------------------------------------------
// readyToCalculate(...)

bool OpManagerEngine::readyToCalculate(double time)
{
  // Check time
  double interval;
  if (m_social_tick > 0.0) {
    interval = 1.0/m_social_tick; 
  } else {
    interval = 1.0;
  }
  
  if ( (time - m_last_iter_time) < interval)
    return(false);

  // Check if we are in a reset
  if ( (time - m_reset_time_cmd) < m_reset_time_interval)
    return(false);
  
  
  return(true);
}

//--------------------------------------------------------------------
// receiveOpinionMessage(...)
// handle a opinon message from others
bool OpManagerEngine::receiveOpinionMessage(std::string optn_msg, double msg_time)
{
  // store the opinons of neighbors

  // first check if we are resetting, if so then ignore this message
  if ( (msg_time - m_reset_time_cmd) < m_reset_time_interval)
    return(true);
  
  // build opinion record
  OpinionRecord new_record;
  bool ok = new_record.setRecordFromMsg(optn_msg, msg_time);
  if (!ok)
    return(false);
  
  std::string vname = new_record.getVname();
  m_neighbor_opinion_records[vname] = new_record;  // overwrites old, assumes unique names
  
  if (m_neighbor_opinion_rec.count(vname)) {
    // Key exists
    m_neighbor_opinion_rec[vname] = m_neighbor_opinion_rec[vname]+1; 
  } else {
    m_neighbor_opinion_rec[vname] = 1; 
  }
  
  
  // update the population status with this new record
  // Must check for old status and remove if needed
  std::string option_with_strongest_opinion = new_record.getStrongestOpinion();

  // if options are valid, but not strongly opinionated, then override
  double strongest_opinion_val = 0.0;
  bool valid_option = new_record.getOpinionForOption(option_with_strongest_opinion, strongest_opinion_val);
  bool over_opinion_thresh = (strongest_opinion_val > m_opinion_thresh);
  if (valid_option && (!over_opinion_thresh))   // valid option is false when option name is ""
      option_with_strongest_opinion = "undecided";
  
  std::map<std::string, std::set<std::string> >::iterator it;
  bool need_to_add_option = true; 
  for (it = m_population_status.begin(); it != m_population_status.end(); it++) {
    std::string option_name = it->first;
    std::set<std::string> agents = it->second;
    
    if(option_name == option_with_strongest_opinion){
      // option already exists
      need_to_add_option = false; 
      if (agents.count(vname)) {
	// agent is already in option list
	// do nothing
      } else {
	// agent is not in the option list, add it
	it->second.insert(vname);
      }
    } else {
      // This is a different option, is the agent in the list for this option?
      // If so, then remove it.
      if (agents.count(vname)) {
	it->second.erase(vname); 
      }
    }     
  }
  if (need_to_add_option && (option_with_strongest_opinion != "")) {
    // create new set and add to the map
    std::set<std::string> new_set;
    new_set.insert(vname);
    m_population_status[option_with_strongest_opinion] = new_set; 
  }
  
  return(true);
}

//--------------------------------------------------------------------
// calculateOwnOpinion(...)
// complete a calcuation of one iteration of own opinion
// provide the time, and an buffer of inputs from the DB.
//                  input_vars_buffer['Option Name'] = option 
bool OpManagerEngine::iterateOwnOpinion(double time)
{
  // Mark when attention is updated, only update once.
  bool attention_updated = false;
  
  // Safety check
  if ( !(this->readyToCalculate(time)) )
      return(false);

  // First determine which options are active
  std::vector<bool> options_status = determineOptionActive();

  std::cout << " options_status = ";
  for (int i = 0; i <options_status.size(); i++) {
    std::cout << boolToString(options_status[i]) << ", ";
  }
  std::cout << ";" << std::endl;
  
  // initialize and book keeping
  double F_ij;
  std::vector<double> F_i;
  std::vector<double> b_i; 

  std::cout << " Starting to iterate through each option" << std::endl;
  // Loop through each option
  for (unsigned int j=0; j<options_status.size(); j++){
    // check if active
    if (!options_status[j]){
      continue;
    }

    // initialize and book keeping
    std::string option_name = m_options[j].getOptionName();

    std::cout << "  -> Updating option j=" << uintToString(j) << " with name =" << option_name << std::endl;
    
    // Steps 1-5: Opinion dynamics 
    //            lines 3a and 3b of arxiv.org/pdf/2009.04332.pdf
    // 
    // Step 1    computes the term alpha_ij * z_ij + sum( gamma_ik * z_kj)
    // Step 2.1  computes the term: beta_ij * z_ij
    // Step 2.2  computes each term in the summation sum( delta_ik * z_kl)
    // Step 2.3  computes the summation sum( S_2 ( ... from Step 2.1 + from Step 2.2 ...) )
    // Step 3    computes the active attention u_i
    // Step 4    computes the input term b_ij
    // Step 5    adds all the terms to compute F_ij
    
    
    // Step 1: Same option coupling
    double term1 = m_options[j].getAlpha() * m_opinions[j];
    std::cout << " Step 1: Calculating Term1" << std::endl;
    
    std::map<std::string, OpinionRecord>::iterator k; 
    for (k=m_neighbor_opinion_records.begin(); k!=m_neighbor_opinion_records.end(); k++){

      double z_kj = 0;
      bool ok = false;
      if ( k->second.isOptionExist(option_name)){
	ok = k->second.getOpinionForOption(option_name, z_kj); 
      } else {
	// this option is not in the neighbors record.
	std::cout << "  This option is not in the neighbors record" << std::endl;
	continue; 
      }
      
      // check if stale or if not ok
      if ( ((time - k->second.getTime()) > m_stale_thresh) || !ok ) {
	std::cout << "  This options was found to be too stale" << std::endl;
	continue; 
      }
      std::cout << "  Found neighbor opinion of z_kj = " << doubleToString(z_kj) << " and the gamma = " << doubleToString( m_options[j].getGamma() ) << std::endl;
      term1 +=  m_options[j].getGamma() * z_kj;
      std::cout << "  Term1 = " << doubleToString(term1) << std::endl; 
    }


    // Step 2  inter-option coupling
    double term2 = 0;
    // For all the beta couplings to other options, find the cooresponding
    // opinion value (state) if the option is active,

    std::cout << " Steps 2.1 - 2.3 Calculating Term2" << std::endl;

    std::map<std::string, double> betaMap = m_options[j].getBetaMap();
    std::map<std::string, double>::iterator l;
    for (l = betaMap.begin(); l != betaMap.end(); l++) {

      // Step 2.1  Calculate the intra agent different opinion for this
      // opinion. 
      double sum3 = 0.0;  // This is for the inner summation
      std::string other_option_name = l->first;
      double beta_il = l->second; 

      std::cout << "  -> Found other option name = " << other_option_name << " with beta_il = " << doubleToString(beta_il) << std::endl;
      // Is this option active on own vehicle? 
      if (options_status[m_option_names_hash[other_option_name]]) {
	std::cout << "     This other option is active on own vehicle with opinion = " << doubleToString(m_opinions[m_option_names_hash[other_option_name]]) << std::endl;
	// add it to the sum
	sum3 += beta_il * m_opinions[m_option_names_hash[other_option_name]];
	std::cout << "     Added it to the sum3, which now =" << doubleToString(sum3) << std::endl;
	
      }

      // Step 2.2 Find any neighbors who have opinions about this other option
      // and add the influence

      // Get the delta weight for this other option, if it exists, then add
      // all the weighed opinions of the neighbors. 
      std::map<std::string, double> deltaMap = m_options[j].getDeltaMap();
      if (deltaMap.count(other_option_name)){
	double delta_ik = deltaMap[other_option_name];

	std::cout << "  -> Found other option on other vehicle = " << other_option_name << " with delta_ik = " << doubleToString(delta_ik) << std::endl;
      
	// Reuse iterator k here:
	for (k=m_neighbor_opinion_records.begin(); k!=m_neighbor_opinion_records.end(); k++){
	  
	  double z_kl = 0;
	  bool ok = false;
	  if ( k->second.isOptionExist(other_option_name)){
	    ok = k->second.getOpinionForOption(other_option_name, z_kl); 
	  } else {
	  // this option is not in the neighbors record.
	    continue; 
	  }
	  
	  // check if stale or if not ok
	  if ( ((time - k->second.getTime()) > m_stale_thresh) || !ok ) {
	    continue; 
	  }

	  std::cout << "     This other option is active on other vehicle with opinion z_kl = " << doubleToString(z_kl) << std::endl;
	  
	  //
	  // Otherwise add it to the sum
	  sum3 += delta_ik * z_kl;
	  std::cout << "     Added it to the sum3, which now =" << doubleToString(sum3) << std::endl;
	}
	
      } // end of if delta exists.  If it does not, then all the contributions from
      // the neighbors opinions are zero anyways, so they are skipped. 


      // Step 2.3  Now add this influence to the outer sum
      std::cout << "     The sigmoid of sum3 is  =" << doubleToString(sigmoid(sum3)) << std::endl;
      term2 += sigmoid(sum3); 

      std::cout << "     Adding it to the term2, which now =" << doubleToString(term2) << std::endl;
      
    }  // end of beta map loop
    
    // Step 3:  Calculate the attention parameter using the dynamic update
    //          Updates the attention parameter m_u_i;
    std::cout << " Step 3:  Updating attention " << std::endl;
    if ( !attention_updated ){
      this->updateAttention(time, options_status);
      attention_updated = true; 
    }
    std::cout << "   Attention was updated to m_u_i = " << m_u_i << std::endl;
    
    // Step 4:  Calculate the input bias for this option
    double b_ij = 0.0;
    std::cout << " Step 4:  Calculating the input bias " << std::endl;
    
    std::string input_var_for_this_option  = m_options[j].getInputVar();
    if (m_info_buff.isKnown(input_var_for_this_option)) {
      // get the value (string only at this time
      bool found_in_buffer = false;
      double dbl_buffer_val = 0.0;
      dbl_buffer_val = m_info_buff.dQuery(input_var_for_this_option, found_in_buffer);
      if ((found_in_buffer) ) {
	std::cout << "   -> Found input variable in buffer = " << doubleToString(dbl_buffer_val) << std::endl;
	b_ij = m_options[j].computeInputB(dbl_buffer_val);
	
	std::cout << "      And computed input b_ij = " << doubleToString(b_ij) << std::endl;
      } else {
	// not found in buffer, or something went wrong
	b_ij = 0.0;
      }
    } else {
      // value not known in the buffer
      b_ij = 0.0; 
    }
    b_i.push_back(b_ij); 

    std::cout << " Step 5:  Adding all terms together " << std::endl;
    std::cout << "    *-> -m_options[j].getDi() * m_opinions[j] = " << doubleToString(-m_options[j].getDi() * m_opinions[j]) << std::endl;
    std::cout << "    *-> b_ij = " << doubleToString(b_ij) << std::endl;
    std::cout << "    *-> m_u_i * ( sigmoid(term1) + term2 ) = " << doubleToString(m_u_i * ( sigmoid(term1) + term2 )) << std::endl;
    

    
    // Step 5:  All terms are calcuated, add together
    F_ij = -m_options[j].getDi() * m_opinions[j] + b_ij + m_u_i * ( sigmoid(term1) + term2 );
    std::cout << "    *-> F_ij = " << doubleToString(F_ij) << std::endl;
    // push back onto the new state vector
    F_i.push_back(F_ij);
    std::cout << " F_i = ";
    for (int i = 0; i <F_i.size(); i++) {
      std::cout << doubleToString(F_i[i]) << ", "; 
    }
    std::cout << ";" << std::endl;
    
      
  } // end of For each option loop


  std::cout << " Last Step:  Projecting and updating " << std::endl;
  // Last step: re-project (rescale) the opinions
  // but only those that are active.  
  std::vector<double> Z_dot_active = projectOp(F_i);
  std::cout << "  -> Z_dot_active = ";
  for (int i = 0; i <Z_dot_active.size(); i++) {
    std::cout << doubleToString(Z_dot_active[i]) << ", ";
  }
  std::cout << ";" << std::endl;
    
  double dt = time - m_last_iter_time;
  std::cout << "    dt = " << doubleToString(dt) << std::endl;

  // Update the opinions using Euler Integration
  // We have a record of which are active via options_status,
  // which is used again here
  double dyn_gain = dynamicGainCalc(Z_dot_active); 
  unsigned int j_offset = 0;
  for (unsigned int j=0; j<options_status.size(); j++){
    // check if active
    if (!options_status[j]){
      // not active - set to zero
      m_opinions[j] = 0.0; 
      continue;
    } else {
      m_opinions[j] = m_opinions[j] + dt * dyn_gain * Z_dot_active[j_offset];
      m_opinion_inputs[j] = b_i[j_offset];
      j_offset++; 
    }
  }
  std::cout << "    ** Updated m_opinions = ";
  for (int i = 0; i <m_opinions.size(); i++) {
    std::cout << doubleToString(m_opinions[i]) << ", ";
  }
  std::cout << ";" << std::endl;

  // double check that the state is in the simplex
  std::vector<double> simplex_opinion_state = projectOp(m_opinions);
  m_opinions = simplex_opinion_state;
  
  
  // Bookkeeping
  m_last_iter_time = time;
  m_opinion_iter_count++;
  
  return(true);
}


//---------------------------------------------------------------
// dynamicGainCalc(z_dot)
//   calculates a gain based on the current value of z_dot
//   and other parameters
//   assumes 
double OpManagerEngine::dynamicGainCalc(const std::vector<double> &z_dot){

  double z_dot_ave = 0.0;
  for (int i=0; i<z_dot.size(); i++)
    z_dot_ave = z_dot_ave + z_dot[i];

  if (z_dot.size() > 0)
    z_dot_ave = z_dot_ave / static_cast<double>(z_dot.size());
  else
    z_dot_ave = 1.0;
			      
  double lower_lim = 0.00001;   // gain is 1.0 at lower limit
  double upper_lim = 0.0001;   // gain is clamped at m_num_int_gain
                            // above this limit
  if (z_dot_ave < lower_lim)
    return(1.0);

  if (z_dot_ave > upper_lim)
    return(m_num_int_gain);

  double delta = z_dot_ave - lower_lim;
  double gain  = ( delta / (upper_lim - lower_lim)) * (1.0 - m_num_int_gain) + m_num_int_gain; 
  return(gain); 
}



//--------------------------------------------------------------------
// buildOwnOpinionMessage
// build an opinion message for the current calculation
// this function would usually be immediately preceeded by
// calculateOwnOpinion(...)
bool OpManagerEngine::buildOwnOpinionMessage(std::string& own_optn_msg)
{
  if ((m_vname == "") || (m_group == ""))
    return(false);
  
  std::string msg = tolower(m_vname) + ":" + tolower(m_group) + ":";

  // add options and associalted opinions
  std::vector<bool> options_status = determineOptionActive();
  if (options_status.size() != m_options.size()){
    // something went wrong
    return(false);
  }

  bool at_least_one_option_active = false;
  
  for (unsigned int i=0; i < options_status.size(); i++) {
    if (!options_status[i])
      continue;

    at_least_one_option_active = true; 
    std::string option_name = m_options[i].getOptionName();
    double dbl_opinion = m_opinions[i];
    std::string str_opinion = doubleToString(dbl_opinion ,4);
    
    msg += tolower(option_name) + "=" + str_opinion;

    if  (i < options_status.size()-1) {
      msg += ":"; 
    }

  }
    
  if (!at_least_one_option_active)
    return(false);
  
  own_optn_msg = msg;
  return(true);
}


//-----------------------------------------------------------------
// Helper function to determine if which options are active,
//        Returns a vector of booleans, each index for one option
//        uses the variable-val buffer. 
std::vector<bool> OpManagerEngine::determineOptionActive()
{
  std::vector<bool> option_status; 
  for (int i = 0; i < m_options.size(); i++) {
    
    // check if this option is active
    if (m_options[i].isAlwaysActive()){
      option_status.push_back(true);
      continue;
    }
    
    // check the buffers to see if the conditions for this option
    // is satisfied
    unsigned int numb_cond = m_options[i].numberOfConditions();
    bool this_option_active = true; 
    
    for (unsigned int j = 0; j < numb_cond; j++) {
      // get this condition
      std::string var_name = m_options[i].getConditionVar(j);
      std::string var_cond_val = m_options[i].getConditionVal(j);
															      
      // see if it is in the buffer.
      if (m_info_buff.isKnown(var_name)) {
	// get the value (string only at this time
	bool found_in_buffer = false;
	std::string buffer_val = "";
	buffer_val = m_info_buff.sQuery(var_name, found_in_buffer);
	if ( !(found_in_buffer) || (buffer_val != var_cond_val) ) {
	  // not found in buffer, or the condition strings not equal
	  this_option_active = false;
	  break; 
	}  
      } else {
	// value not known in the buffer
	this_option_active = false;
	break;
      }
      
    } // end of for each condition loop
    
    // If no false conditions were found, mark this as active
    option_status.push_back(this_option_active);

    // If option is not active, the zero out the opinion
    // so that it does not retain the history if it does become
    // active again.
    if (!this_option_active){
      // sanity check
      if (i < m_opinions.size()){
	m_opinions[i] = 0.0; 
      }
    }
    
    
  } // end of for each option loop
  
  return(option_status);
}

// numberOptionsActive
int OpManagerEngine::numberOptionsActive()
{
  std::vector<bool> option_status = determineOptionActive();

  int count = 0;
  std::vector<bool>::iterator it;
  for (it = option_status.begin(); it != option_status.end(); it++){
    if(*it)
      count += 1;
  }
  return(count);
}



// Projection operator
// Definition in arxiv.org/pdf/2009.04332.pdf
std::vector<double> OpManagerEngine::projectOp(const std::vector<double> in)
{
  std::vector<double> out;
  unsigned int N_o = in.size();
  if (N_o == 0) {
    return(out); 
  }
  double dbl_N_o = static_cast<double>(N_o);
  double dbl_N_o_inv = 1/dbl_N_o;  // cannot be zero; 
  double temp_val; 

  for (unsigned int i=0; i<N_o; i++) {
    temp_val = 0;
    for (unsigned int j=0; j<N_o; j++) {
      if (i==j)
	temp_val += (1.0 - dbl_N_o_inv) * in[j];
      else
	temp_val -= dbl_N_o_inv * in[j]; 
    }
    out.push_back(temp_val);
  }
  return(out);
}

//--------------------------------------------------
// Procedure:  Update attention
//             Inputs: current time as double
//                     options_status is a vector of bools
//                     which indicate which options are active
void OpManagerEngine::updateAttention(double time, std::vector<bool> options_status)
{
  // Estimate u_a and u_d from current records.
  //estBoundsUaUd();  // Not used at this time

  
  double sum1 = 0.0;
  double count = 0.0;
  // add the square of all own ACTIVE opinions
  for (unsigned int i = 0; i < m_opinions.size(); i++){
    if (options_status[i]){
      sum1  += m_opinions[i]*m_opinions[i];
      count += 1.0; 
    }
  }
  // Normalize
  sum1 = sum1 / count;
  
  std::cout << " Updating attention:  Normalized square of own opinions is = " << doubleToString(sum1) << std::endl;
  
  std::map<std::string, OpinionRecord>::iterator it;
  for (it = m_neighbor_opinion_records.begin(); it != m_neighbor_opinion_records.end(); it++) {

    // check if this record stale
    bool stale = ( (time - it->second.getTime() ) > m_stale_thresh);
    if (!stale) {
      sum1 += it->second.getOpinionsSquared(); 
    }
    std::cout << " -> Checked vehicle " << it->second.getVname() << " and running sum of squares is now = " << doubleToString(sum1) << std::endl;
  }

  // Update attention
  double dt = time - m_last_iter_time;
  std::cout << " Updating attention:  dt = " << doubleToString(dt) << std::endl;
  std::cout << " Updating attention:  m_u_min + (m_u_max - m_u_min) * satFunctionHill(sum1) = " << m_u_min + (m_u_max - m_u_min) * satFunctionHill(sum1) << std::endl;
  
  double du_dt = (-1.0 * m_u_i + m_u_min + (m_u_max - m_u_min) * satFunctionHill(sum1)) / m_tau_u;
  std::cout << " Updating attention: satFunctionHill(sum1) = " << doubleToString(satFunctionHill(sum1)) << std::endl;
  m_u_i = m_u_i + du_dt * dt;
  std::cout << " Updating attention:  m_u_i = " << doubleToString(m_u_i) << std::endl;

  // clip attention
  if (m_u_i > m_u_max)
    m_u_i = m_u_max;

  if (m_u_i < m_u_min)
    m_u_i = m_u_min;

  return; 
}

//---------------------------------------------------------------
// Estimate bounds for u_a and u_d
// Definition in arxiv.org/pdf/2009.04332.pdf
// Equations 12 and 13
void OpManagerEngine::estBoundsUaUd() {

  // Calculate lambda max and lambda min
  // TODO
  double lambda_max = 1;
  double lambda_min = 1; 

  std::vector<Option>::iterator op_it;
  for (op_it = m_options.begin(); op_it != m_options.end(); op_it++) {

    double alpha = op_it->getAlpha(); 
    double gamma = op_it->getGamma();

    std::map<std::string, double> delta_map = op_it->getDeltaMap();
    double max_delta = std::numeric_limits<double>::lowest(); 
    double min_delta = std::numeric_limits<double>::max();

    std::map<std::string, double>::iterator map_it;
    for (map_it = delta_map.begin(); map_it != delta_map.end(); map_it++) {

      if (map_it->second < min_delta)
	min_delta = map_it->second;

      if (map_it->second > max_delta)
	max_delta = map_it->second; 
    }

    
    std::map<std::string, double> beta_map = op_it->getBetaMap();
    double max_beta = std::numeric_limits<double>::lowest(); 
    double min_beta = std::numeric_limits<double>::max();

    for (map_it = delta_map.begin(); map_it != delta_map.end(); map_it++) {

      if (map_it->second < min_beta)
	min_beta = map_it->second;

      if (map_it->second > max_beta)
	max_beta = map_it->second; 
    }

    // with all alphas, gamma, and bounds for delta and beta we can
    // estimate bounds for u_a and u_d;
    // TODO
    

  }
  

  return; 
}



//-----------------------------------------------------------------
// Get outputs if appropriate:
//           get the outputs from the most favorable option
//           returns false if no strong opinion, or something went wrong
//           updates values by reference

bool OpManagerEngine::getOutputsForLargestOpinion(std::vector<std::string>& names, std::vector<std::string>& vals)
{
  // Step 1: Find the option with the highest opinion
  unsigned int best_index = 0;
  double highest_opinion = 0.0;   // Opinions are in the simplex
  
  for ( unsigned int i = 0; i < m_opinions.size(); i++) {
    if (m_opinions[i] > highest_opinion) {
      best_index = i;
      highest_opinion = m_opinions[i];
    }
  }
  m_strongest_opinion = m_options[best_index].getOptionName();
  
  // Step 2: Are we strongly opinionated?
  if (highest_opinion < m_opinion_thresh)
    return(false); 

  // Step 3:  Otherwise get the output names and vals
  std::vector<std::string> names_temp;
  std::vector<std::string> vals_temp;

  unsigned int numb_outputs = m_options[best_index].getOptionOutputs(names_temp, vals_temp);

  // Final check before returning
  if (numb_outputs <=0)
    return(false);

  names = names_temp; 
  vals = vals_temp;
  return(true); 
}



//-----------------------------------------------------------------
// Build a report
std::list<std::string> OpManagerEngine::getReport(double time_now)
{
  std::list<std::string> report;
  std::string pd_vname  = padString(m_vname, 5, false);
  std::string pd_vgroup = padString(m_group, 8, false);
  std::string str_attention_val = doubleToString(m_u_i,3);
  std::string pd_attention      = padString(str_attention_val, 7, false); 
  std::string str_opinion_iter_count = uintToString(m_opinion_iter_count);
  
  report.push_back("Name: " + pd_vname + " Group: " + pd_vgroup + " Atten: " + pd_attention + " Opinion Iter: " + str_opinion_iter_count);
  report.push_back("=============================================================");
  report.push_back(" Option              |  Own Opinion   |  Input  | Active? ");
  report.push_back("-------------------------------------------------------------");

  std::vector<bool> options_status = determineOptionActive();
  
  if (m_options.size() == m_opinions.size()) {
    for(unsigned int i = 0; i < m_options.size(); i++) {
      std::string pd_option_name = padString(m_options[i].getOptionName(), 20, false); 
      std::string str_opinion_val = doubleToString(m_opinions[i], 4);
      std::string pd_opinion_val = padString(str_opinion_val, 10, false);
      std::string str_opinion_input_val = doubleToString(m_opinion_inputs[i], 3);
      std::string pd_opinion_input_val = padString(str_opinion_input_val, 6, false);

      std::string str_op_status = " Yes ";
      if (!options_status[i]){
	str_op_status = " No  ";
	pd_opinion_val = " N/A      ";
      }
      
      report.push_back( " " + pd_option_name + ":  " + pd_opinion_val + "    :  " + pd_opinion_input_val + " : " + str_op_status);
    }
  } else {
    report.push_back("ERROR:  BAD OPINION STATE");
    
  }

  report.push_back("                                                             ");
  report.push_back("=============================================================");
  report.push_back("== Population State                 =========================");
  report.push_back("=============================================================");
  report.push_back(" Option Name       | Agents  ");
  report.push_back("-------------------------------------------------------------");

  std::map<std::string, std::set<std::string> >::iterator p_it;
  for (p_it = m_population_status.begin(); p_it != m_population_status.end(); p_it++) {
    std::string option_name  = p_it->first;
    std::string pd_option_name = padString(option_name, 18, false);

    std::string line = " " + pd_option_name + ": "; 

    // Add own name if appropriate
    bool added_own_name = false; 
    if (option_name == m_strongest_opinion) {
      line += m_vname + ",";
      added_own_name = true;
    }

    // Add names of neighbors. 
    std::set<std::string> set_of_agents = p_it->second;
    std::set<std::string>::iterator sp_it;
    for (sp_it = set_of_agents.begin(); sp_it != set_of_agents.end(); sp_it++) {
      // check if record is too stale to know anything for sure
      if ( (time_now - m_neighbor_opinion_records[*sp_it].getTime()) < m_stale_thresh )
	line += *sp_it + ","; 
    }
    if ( (set_of_agents.size() > 0) || added_own_name){
      // at least one agent is in the set, remove the last comma
      line = line.substr(0, line.size()-1);
    }
    report.push_back(line); 
  }

  report.push_back("                                                             ");
  report.push_back("=============================================================");
  report.push_back("== Neighbor Opinions                =========================");
  report.push_back("=============================================================");
      
  report.push_back(" Name  | Count | Time Past | Last recieved msg    ");
  report.push_back("-------------------------------------------------------------");
  
  std::map<std::string, OpinionRecord>::iterator rec_it;
  for (rec_it=m_neighbor_opinion_records.begin(); rec_it!=m_neighbor_opinion_records.end(); rec_it++) {
    std::string neighbor_name = rec_it->second.getVname();
    std::string neighbor_count = uintToString(m_neighbor_opinion_rec[neighbor_name]);
    double time_past = time_now - rec_it->second.getTime(); 
    std::string neighbor_time  = doubleToString(time_past, 2);
    
    std::string pd_neighbor_name = padString(neighbor_name, 5, false);
    std::string pd_neighbor_count = padString(neighbor_count, 5, false);
    std::string pd_neighbor_time  = padString(neighbor_time, 9, false);
    std::string pd_last_msg = padString(rec_it->second.getLastRecordMsg(), 20, false);

    
    report.push_back( " " + pd_neighbor_name + " : " + pd_neighbor_count + " : "
		      + pd_neighbor_time + " : " + pd_last_msg);
  }


  return(report);
}



//----------------------------------------------
//  getUpdatedOptionsStateLists
//    This function gets the updated lists describing which vehicles
//    are on each option. Variables are updated by reference.
//    

bool OpManagerEngine::getUpdatedOptionStateLists(std::vector<std::string>& opt_names, std::vector<std::string>& agent_list, double time_now) {

  std::vector<std::string> names_ret;
  std::vector<std::string> list_ret; 

  std::map<std::string, std::set<std::string> >::iterator p_it;
  for (p_it = m_population_status.begin(); p_it != m_population_status.end(); p_it++) {
    names_ret.push_back(p_it->first);

    // Add names of neighbors.
    std::string this_list = ""; 
    std::set<std::string> set_of_agents = p_it->second;
    std::set<std::string>::iterator sp_it;
    for (sp_it = set_of_agents.begin(); sp_it != set_of_agents.end(); sp_it++) {
      if ( (time_now - m_neighbor_opinion_records[*sp_it].getTime()) < m_stale_thresh )
	this_list += *sp_it + ","; 
    }
    if ( set_of_agents.size() > 0){
      // at least one agent is in the set, remove the last comma
      this_list = this_list.substr(0, this_list.size()-1);
    }
    list_ret.push_back(this_list); 
  }

  opt_names = names_ret;
  agent_list = list_ret; 

  return(true);
}
