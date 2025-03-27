#include "xCounter.h"

using namespace std;
#include <cstdio>
#include <string.h>
//---------------------------------------------------------
// EvalEngine Standard Kernel: xCounterKernel
/*
    Counts the number of updates published to a variable
*/

xCounterKernel::xCounterKernel(std::string identity, std::string suffix)
{

    m_identity = identity;

    // variables
    m_counter = 0;

    // Subscriptions
    //TODO: Find an even more generic moos variable which is always present
    m_sub = "NAV_X";
    
    // Parameters
    m_sub_key = "var_to_count"+suffix;
    m_pub_key = "pub_to_var"+suffix;

    // Publications
    m_pub = "X_COUNTER";

    // Include the subscriptions which describe the needs of this kernel to the EvalEngine
    m_subscriptions.insert(m_sub);

    // Include the publications which describe the needs of this kernel to the EvalEngine
    m_publications.insert(m_pub);

    // Include the parameters which describe the needs of this kernel to the EvalEngine, and their default values
    m_parameters[m_sub_key] = m_sub;
    m_parameters[m_pub_key] = m_pub;
};

// Sets the parameters that has been received from a file
bool xCounterKernel::SetParams(const map<string, string> &params)
{
    // Remapping inputs
    if (params.count(m_sub_key))
    {
        updateSubscriptions(m_sub, params.at(m_sub_key));
        m_sub = params.at(m_sub_key);
    }

    if (params.count(m_pub_key))
    {
        m_pub = params.at(m_pub_key);
    }
    return true;
}

// From the message buffer which is
bool xCounterKernel::UpdateVariable(const EEMessage &message)
{
    if (message.getVarName() == m_sub)
    {
        m_counter++;
    }
    return true;
}

// On a pass over, it can post updates to an object maintained by the EvalEngine
bool xCounterKernel::Evaluate(std::vector<std::pair<std::string, std::string> > &updates, std::vector<std::string> &kernel_level_warnings)
{
    //updates[m_pub] = to_string(m_counter);
    updates.push_back(std::make_pair(m_pub,to_string(m_counter)));
    return true;
}

// A kernel can make a contribution to the EvalEngine's AppCasting table
bool xCounterKernel::AppCastingNotice(ACTable &actab)
{
    const uint32_t msg_buff_len = 128;
    char message[msg_buff_len];
    memset(message, msg_buff_len, '\0');
    snprintf(message, msg_buff_len, "Updates to '%s' received: %d\n", m_sub.c_str(), m_counter);
    actab << m_identity << message;
    return true;
}
