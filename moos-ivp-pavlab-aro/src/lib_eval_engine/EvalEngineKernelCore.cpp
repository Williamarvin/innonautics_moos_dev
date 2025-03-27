
#include "EvalEngineKernelCore.h"

EvalEngineKernel::EvalEngineKernel(){};
EvalEngineKernel::~EvalEngineKernel(){};

std::set<std::string> EvalEngineKernel::get_subscriptions()
{
    return m_subscriptions;
}

bool EvalEngineKernel::updateSubscriptions(std::string old_val, std::string new_val)
{
    if (m_subscriptions.count(old_val) == 0)
        return false;
    m_subscriptions.erase(old_val);
    m_subscriptions.insert(new_val);
    return true;
}

std::map<std::string, std::string> EvalEngineKernel::get_parameters()
{
    return m_parameters;
}

bool EvalEngineKernel::updatePublications(std::string old_val, std::string new_val)
{
    if (m_publications.count(old_val) == 0)
        return false;
    m_publications.erase(old_val);
    m_publications.insert(new_val);
    return true;
}

void EvalEngineKernel::set_debug_stream(char debug_stream_name[])
{
    m_debug_stream_name = debug_stream_name;
}

/*
    These following functions are defined by the user and the kernel which inherits from this base class

    > virtual bool EvalEngineKernel::SetParams(const std::map<std::string, std::string> &params){};

    > virtual bool EvalEngineKernel::UpdateVariable(const EEMessage &message){};

    > virtual bool EvalEngineKernel::Evaluate(std::map<std::string, std::string> &updates){};

    > virtual bool EvalEngineKernel::AppCastingNotice(ACTable &actab){};
*/

std::string EvalEngineKernel::generateRunWarning(std::string message) {
    return "<"+m_identity+"> : " + message;
}

bool EvalEngineKernel::dbg_print(const char *format, ...)
{
    if (m_debug_stream_name != nullptr)
    {
        va_list args;
        va_start(args, format);
        m_debug_stream = fopen(m_debug_stream_name, "a");
        if (m_debug_stream != nullptr)
        {
            vfprintf(m_debug_stream, format, args);
            fclose(m_debug_stream);
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}