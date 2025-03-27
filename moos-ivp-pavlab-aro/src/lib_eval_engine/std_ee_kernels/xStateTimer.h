#pragma once
#include "EvalEngine.h"
#include <string.h>
#include <vector>

class xStateTimer : public EvalEngineKernel
{
private:
    // State variables
    double m_ctimer;
    double m_last_time;
    double m_timenow;
    double m_delta_t;
    bool m_timing;
    std::string m_var_to_time_latest_val;
    uint32_t m_nresets;
    bool m_reset;
    bool m_transitioned_into_timing;

    // Parameter defaults
    std::string m_isCummulative;
    std::string m_var_condition;
    std::string m_sub_state_expr;

    // Subscriptions & publications
    std::string m_sub_moos_time;
    std::string m_sub_var_to_time;
    std::string m_sub_var_reset;
    std::string m_pub_time_to_var;
    std::string m_pub_nresets_to_var;

    // Parameter keys
    std::string m_sub_var_to_time_key;
    std::string m_sub_var_reset_key;
    std::string m_var_condition_key;

    //TODO: This is for only a single expression right now
    std::string m_sub_state_expr_key;
    std::string m_sub_state_val_key;
    std::string m_pub_time_to_var_key;
    std::string m_pub_nresets_to_var_key;
    std::string m_cummulative_key;

public:
    //Sets the subscriptions and the parameters that it will consume from the configuration file.
    //The EvalEngine core will reference these variables
    xStateTimer(std::string identity = "xStateTimer", std::string suffix = "");

    ~xStateTimer();

    //Sets the parameters that has been received from a file
    bool SetParams(const std::map<std::string, std::string> &params);

    //From the message buffer which is 
    bool UpdateVariable(const EEMessage &message);

    //On a pass over, it can post updates to an object maintained by the EvalEngine
    bool Evaluate(std::vector<std::pair<std::string, std::string> > &updates, std::vector<std::string> &kernel_level_warnings);

    //A kernel can make a contribution to the EvalEngine's AppCasting table
    bool AppCastingNotice(ACTable &actab);
};
