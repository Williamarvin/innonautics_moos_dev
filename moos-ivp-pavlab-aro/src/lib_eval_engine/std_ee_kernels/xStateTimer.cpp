#include "xStateTimer.h"

using namespace std;
#include <cstdio>
#include <string.h>
#include "MBUtils.h"

//---------------------------------------------------------
// EvalEngine Standard Kernel: xStateTimer
/*
    Monitors how long some variable is held, i.e.
    - DEPLOY=TRUE
    - SURVEY=TRUE
    etc..
        
    Considerations
        - Timer can be cummulative or reset when it exits a state
            - i.e. Total time deployed, or how long since the last time deploy was published to
        - Timer can be cummulative and receive force a reset condition. This condition can also be applied to a non-cummulative state timer
            - i.e. uPokeDB alpha.moos++ XSTATETIMER_RESET:=true
            -> XSTATETIMER_NRESETS+=1 ...
        - The state which it is tracking can be collection of conditions
            - VAR=NAME
            - NOT IMPLEMENTED - VAR1=NAME1 && VAR2=NAME2, VAR1=NAME1 || VAR2=NAME2 (Downstream)
            - NOT IMPLEMENTED - VAR > NUM, VAR < NUM, VAR >= NUM, VAR <= NUM (Downstream)
*/

xStateTimer::xStateTimer(std::string identity, std::string suffix)
{

    m_identity = identity;

    // State variables and default values
    m_ctimer = 0.00;
    m_last_time = 0.00;
    m_timenow = 0.00;
    m_delta_t = 0.00;
    m_nresets = 0;
    m_transitioned_into_timing = false;
    m_sub_state_expr = "==";

    m_var_to_time_latest_val = "false";

    // Parameter defaults
    m_isCummulative = "true";
    m_var_condition = "TRUE";

    // Parameters
    m_sub_var_to_time_key = "var_to_time"+suffix;
    m_var_condition_key = "var_condition"+suffix;
    m_sub_var_reset_key = "timer_reset_condition"+suffix;
    m_pub_time_to_var_key = "pub_time_to_var"+suffix;
    m_pub_nresets_to_var_key = "pub_nresets_to_var"+suffix;
    m_cummulative_key = "timer_is_cummulative"+suffix;

    // Subscriptions & publications
    m_sub_moos_time = "MOOSTime";
    m_sub_var_to_time = "DEPLOY";
    m_sub_var_reset = toupper(m_identity) + "_RESET";
    m_pub_time_to_var = toupper(m_identity) + "_TIME";
    m_pub_nresets_to_var = toupper(m_identity) + "_NRESETS";

    // Include the subscriptions which describe the needs of this kernel to the EvalEngine
    m_subscriptions.insert(m_sub_var_to_time);
    m_subscriptions.insert(m_sub_var_reset);
    m_subscriptions.insert(m_sub_moos_time);

    // Include the publications which describe the needs of this kernel to the EvalEngine
    m_publications.insert(m_pub_time_to_var);
    m_publications.insert(m_pub_nresets_to_var);

    // Include the parameters which describe the needs of this kernel to the EvalEngine, and their default values
    m_parameters[m_sub_var_to_time_key] = m_sub_var_to_time;
    m_parameters[m_var_condition_key] = m_var_condition;
    m_parameters[m_sub_var_reset_key] = m_sub_var_reset;
    m_parameters[m_pub_time_to_var_key] = m_pub_time_to_var;
    m_parameters[m_pub_nresets_to_var_key] = m_pub_nresets_to_var;
    m_parameters[m_cummulative_key] = m_isCummulative;
};

// Sets the parameters that has been received from a file
bool xStateTimer::SetParams(const map<string, string> &params)
{
    if (params.count(m_sub_var_to_time_key))
    {
        updateSubscriptions(m_sub_var_to_time, params.at(m_sub_var_to_time_key));
        m_sub_var_to_time = params.at(m_sub_var_to_time_key);
    }

    if (params.count(m_var_condition_key))
    {
        updateSubscriptions(m_var_condition, params.at(m_var_condition_key));
        m_var_condition = tolower(params.at(m_var_condition_key));
    }

    if (params.count(m_sub_var_reset_key))
    {
        updateSubscriptions(m_sub_var_reset, params.at(m_sub_var_reset_key));
        m_sub_var_reset = params.at(m_sub_var_reset_key);
    }

    if (params.count(m_pub_time_to_var_key))
    {
        updatePublications(m_pub_time_to_var, params.at(m_pub_time_to_var_key));
        m_pub_time_to_var = params.at(m_pub_time_to_var_key);
    }

    if (params.count(m_pub_nresets_to_var_key))
    {
        updatePublications(m_pub_nresets_to_var, params.at(m_pub_nresets_to_var_key));
        m_pub_nresets_to_var = params.at(m_pub_nresets_to_var_key);
    }
    if (params.count(m_cummulative_key))
    {
        m_isCummulative = params.at(m_cummulative_key);
    }

    return true;
}

// From the message buffer which is
bool xStateTimer::UpdateVariable(const EEMessage &message)
{
    bool captured_update = false;
    if (message.getVarName() == m_sub_var_to_time)
    {
        m_var_to_time_latest_val = tolower(message.msg());
        MOOSTrimWhiteSpace(m_var_to_time_latest_val);
        captured_update = true;
    }
    else if (message.getVarName() == m_sub_var_reset)
    {
        if (tolower(message.msg()) == "true")
            m_reset = true;
        captured_update = true;
    }
    else if (message.getVarName() == m_sub_moos_time)
    {
        m_timenow = stod(message.msg());
        captured_update = true;
    }
    return captured_update;
}

// On a pass over, it can post updates to an object maintained by the EvalEngine
bool xStateTimer::Evaluate(std::vector<std::pair<std::string, std::string> > &updates, std::vector<std::string> &kernel_level_warnings)
{
    //If it is time to reset, then we reset the timer
    if (m_reset == true)
    {
        m_ctimer = 0;
        m_reset = false;
        m_nresets++;
    }
    // If the condition is currently met
    //TODO: We can consider a collection of logic conditions

    if (tolower(m_var_to_time_latest_val) == tolower(m_var_condition))
    {
        // We check to see if this is our first satisfying condition, and if we are transitioning into a timing state
        if (m_transitioned_into_timing == false)
        {
            // If we are transitioning into a valid state, then all we can do is save the last valid time
            m_transitioned_into_timing = true;
            m_last_time = m_timenow;
        }
        // If we already transitioned into a timing state and have a valid last saved time since when the condition has been met
        //  we then observe the time delta and apply it to our timer
        else
        {
            m_delta_t = m_timenow - m_last_time;
            m_ctimer += m_delta_t;
            m_last_time = m_timenow;
        }
    }
    else
    {
        // Otherwise, the condition is not met, and we are transitioning out of a timing state or holding the timer at 0 if it is not cummulative
        m_transitioned_into_timing = false;
        if (m_isCummulative == "false")
            m_ctimer = 0.00;
    }
    //updates[m_pub_time_to_var] = to_string(m_ctimer);
    updates.push_back(std::make_pair(m_pub_time_to_var,to_string(m_ctimer)));
    //updates[m_pub_nresets_to_var] = to_string(m_nresets);
    updates.push_back(std::make_pair(m_pub_nresets_to_var,to_string(m_nresets)));
    return true;
}

// A kernel can make a contribution to the EvalEngine's AppCasting table
bool xStateTimer::AppCastingNotice(ACTable &actab)
{
    const uint32_t msg_buff_len = 128;
    char message[msg_buff_len];
    memset(message, msg_buff_len, '\0');
    snprintf(message, msg_buff_len, "Timer status: %s %s %s for %0.2f seconds with %d resets\n", m_sub_var_to_time.c_str(), m_sub_state_expr.c_str(), m_var_condition.c_str(), m_ctimer, m_nresets);
    actab << m_identity << message;
    return true;
}
