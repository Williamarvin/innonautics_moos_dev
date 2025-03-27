/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionManagerEngine.h                          */
/*    DATE: March 20th, 2023                                */
/************************************************************/

#ifndef OpManagerEngine_HEADER
#define OpManagerEngine_HEADER

#include "Option.h"
#include "OpinionRecord.h"
#include <map>
#include <vector>
#include <fstream>  // to read config block
#include "InfoBuffer.h" // for info buffer
#include <iostream>
#include <math.h>  // for exp
#include <limits>  // for numeric limits




class OpManagerEngine
{
 public:
  OpManagerEngine();
  ~OpManagerEngine();

  // Setup functions:
  bool setTauU(double val)     {m_tau_u = val; return(true);};
  bool setUth(double val)      {m_u_th = val; return(true);};
  bool setSatFunOrder(double val) {m_Hill_order = val; return(true);};
  bool setMinAtten(double val) {m_u_min = val; m_u_i = val; return(true);};
  bool setMaxAtten(double val) {m_u_max = val; return(true);};
  bool setNumIntGain(double val) {m_num_int_gain = val; return(true);};
  
  
  bool setName(std::string val) {m_vname = val; return(true);};
  bool setGroup(std::string val) {m_group = val; return(true);};
  bool setStaleThresh(double val) {m_stale_thresh = val; return(true);};
  bool setSocialTick(double val) {if (val >0){m_social_tick = val; return(true);} else {return(false);}};
  bool setOpinionThresh(double val) {m_opinion_thresh = val; return(true);};

  bool handleOptConfig(std::string filename, std::string& warn_msg);
  bool setupOpinionManager(double time);

  // Getters
  std::vector<std::string> getVarsToRegister() {return(m_vars_to_register);};
  std::list<std::string> getReport(double time);
  double getAttentionU()  {return(m_u_i);};
  bool   getUpdatedOptionStateLists(std::vector<std::string>& opt_names, std::vector<std::string>& agent_list, double time_now);

  
  // Buffer and message handling
  //----------------------------//

  bool loadValIntoBuffer(std::string var, std::string val) {return( m_info_buff.setValue(var, val) );};
  bool loadValIntoBuffer(std::string var, double val) {return( m_info_buff.setValue(var, val) );};
  bool isValInBuffer(std::string var); 
  
  // build an opinion message for the current calculation
  // this function would usually be immediately preceeded by
  // calculateOwnOpinion(...)
  bool buildOwnOpinionMessage(std::string& own_optn_msg);

  // handle a opinon message from others
  bool receiveOpinionMessage(std::string optn_msg, double msg_time);

  
  // Operational functions:
  //------------------------//
  // check if ready to calcuate, returns true if so.
  bool readyToCalculate(double time);
  bool resetOpinionState(double time, double reset_time=5.0); 


  // complete a calcuation of one iteration of own opinion
  // provide the time, and an buffer of input values from the DB. 
  bool iterateOwnOpinion(double time);

  // get the outputs from the most favorable option
  // returns false if no strong opinion, or something went wrong
  // updates values by reference
  bool getOutputsForLargestOpinion(std::vector<std::string>& names, std::vector<std::string>& vals); 

  int numberOptionsActive(); 


 private:

  // Operational Functions
  std::vector<bool> determineOptionActive();
  std::vector<double> projectOp(const std::vector<double> in);
  double sigmoid(double val) {return( tanh(val) );};
  double satFunctionHill(double y) {return(  pow(y,m_Hill_order) /
					     ( pow(m_u_th,m_Hill_order) + pow(y,m_Hill_order))); }; 
  void updateAttention(double time, std::vector<bool> options_status); 
  void estBoundsUaUd();
  double dynamicGainCalc(const std::vector<double> &z_dot); 

  // Dyamical parameters
  double m_u_a_upper_bound;
  double m_u_a_lower_bound; 
  double m_u_d_upper_bound;
  double m_u_d_lower_bound; 

  double m_tau_u;        // Attention timescale
  double m_u_min;        // Minimum attention
  double m_u_max;        // Maximum attention  robustness to input
  double m_u_th;         // Sensitivity to input
  double m_Hill_order;   // Sensitivity to input
  double m_opinion_thresh;
  double m_num_int_gain; 
  

  // Bookkeeping states, etc. 
  bool m_setup_complete;
  double m_last_iter_time;
  double m_social_tick;
  double m_stale_thresh;
  double m_reset_time_interval;
  double m_reset_time_cmd;
  
  std::string m_vname;
  std::string m_group;
  unsigned int m_opinion_iter_count; 

  // Var buffer to hold all variables
  // Input variables, ex DANGER_LEVEL = 100
  // Condition variables: ex EXPLORE = true
  // Output vars are not required in the buffer at this time

  // The buffer will store them
  InfoBuffer m_info_buff;

  // Additionally, a vector of all variables to register for
  // across all options. 
  std::vector<std::string> m_vars_to_register; 


  // States: own options and opinions. 
  std::vector<Option> m_options;
  std::map<std::string, unsigned int> m_option_names_hash;
  std::vector<double> m_opinions;
  std::vector<double> m_opinion_inputs; 
 

  // Attention
  double m_u_i;

  // States: options and opinions of others. 
  // first index is the vname; 
  std::map<std::string, OpinionRecord> m_neighbor_opinion_records;
  std::map<std::string, unsigned int> m_neighbor_opinion_rec;


  // Set of options and the vehicles in the population that are
  // strongly opinionated about those options
  std::map<std::string, std::set<std::string> > m_population_status;
  // For convienence, 
  std::string m_strongest_opinion;
 
};
  



#endif 
