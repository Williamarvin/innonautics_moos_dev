/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Option.h                                        */
/*    DATE: March 20th, 2023                                */
/************************************************************/

#ifndef Option_HEADER
#define Option_HEADER

#include <vector>
#include <string>
#include <map>
#include <random>  // for random perturb
#include <chrono>  // for random perturb
#include "MBUtils.h"


class Option
{
 public:
  Option();
  ~Option();

  bool checkOptionOk(std::string& warn_msg);
  bool needToRegister(std::string& var);

  // Setters
  bool setName(std::string val) {m_name = val; return(true);};
  bool setOptionOutput(std::string var, std::string val);
  bool setResistanceWeight(double val) {m_resistance_weight = val; return(true);};

  bool setSocialMode(std::string val);
  
  bool setIntraAgentSameOptionCoupling(double val) {m_alpha = val; return(true);};
  bool setIntraAgentInterOptionCoupling(std::string);  // beta
  bool setInterAgentSameOptionCoupling(double val) {m_gamma = val; return(true);};
  bool setInterAgentInterOptionCoupling(std::string);  // delta

  bool setInputVar(std::string val) {m_input_var = toupper(val); return(true);};
  bool setInputFunType(std::string val);
  bool setMaxInputVal(double val) {m_max_input_val = val; return(true);};
  bool setMinInputVal(double val) {m_min_input_val = val; return(true);};
  bool setInputGain(double val) {m_input_gain = val; return(true);};
  
  bool setActiveCondition(std::string var, std::string val);
  bool setRandPerturb(bool val) {m_random_perturb_params = val; return(true);};

  // Getters
  unsigned int getNumberOutputs() { return(m_option_output_name.size());};
  unsigned int getOptionOutputs(std::vector<std::string>& names, std::vector<std::string>& vals);
  
  std::string getOptionName() {return(m_name);};
  
  bool isAlwaysActive() {return(m_always_active);};
  unsigned int numberOfConditions() {return(m_active_condition_name.size());};
  std::string getConditionVar(unsigned int indx);
  std::string getConditionVal(unsigned int indx);
  std::string getInputVar() {return(m_input_var);};

  double getDi()   {return(m_resistance_weight);};
  double getAlpha() {return(m_alpha);};
  std::map<std::string, double> getBetaMap() {return(m_beta);};
  double getGamma() {return(m_gamma);};
  std::map<std::string, double> getDeltaMap() {return(m_delta);};

  double computeInputB(double raw_input_val); 
  
 private:

  std::string m_name;
  std::vector<std::string> m_option_output_name;
  std::vector<std::string> m_option_output_val;

  std::vector<std::string> m_active_condition_name;
  std::vector<std::string> m_active_condition_val;
  bool m_always_active;

  // own resistance weight
  double m_resistance_weight;

  // social weights
  std::string m_social_mode;  // normal, cooperative, competitive, oblivious  
  double m_alpha;
  double m_gamma;

  std::map<std::string, double> m_beta; 
  //std::vector<std::string> m_beta_option_names;
  //std::vector<double> m_beta_option_vals;

  std::map<std::string, double> m_delta; 
  //std::vector<std::string> m_delta_option_names;
  //std::vector<double> m_delta_option_vals;

  // randomly perturb param?
  bool m_random_perturb_params;
  
  // input variables
  std::string m_input_var;
  std::string m_input_fun_type;
  std::string m_input_fun_linear_spec;
  double m_min_input_val;
  double m_max_input_val;
  double m_input_gain; 

    
};
  
Option buildOptionFromSpec(std::list<std::string> spec, bool& ok, std::string& warn_msg);


#endif
