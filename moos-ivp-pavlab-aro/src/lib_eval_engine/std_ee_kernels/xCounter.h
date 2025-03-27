#pragma once
#include "EvalEngine.h"
#include <string.h>
#include <vector>

class xCounterKernel : public EvalEngineKernel
{
private:
    // State variables and default values
    uint32_t m_counter;

    // Parameter defaults
    
    
    // Subscriptions
    std::string m_sub;
    
    // Parameters
    std::string m_sub_key;
    std::string m_pub_key;

    // Publications
    std::string m_pub;

public:
    //Sets the subscriptions and the parameters that it will consume from the configuration file.
    //The EvalEngine core will reference these variables
    xCounterKernel(std::string identity = "xCounter", std::string suffix = "");

    ~xCounterKernel();

    //Sets the parameters that has been received from a file
    bool SetParams(const std::map<std::string, std::string> &params);

    //From the message buffer which is 
    bool UpdateVariable(const EEMessage &message);

    //On a pass over, it can post updates to an object maintained by the EvalEngine
    bool Evaluate(std::vector<std::pair<std::string, std::string> > &updates, std::vector<std::string> &kernel_level_warnings);

    //A kernel can make a contribution to the EvalEngine's AppCasting table
    bool AppCastingNotice(ACTable &actab);
};
