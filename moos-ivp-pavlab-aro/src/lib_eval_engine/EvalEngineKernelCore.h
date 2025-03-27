#pragma once

#include <string>
#include <map>
#include <set>
#include "EvalEngineMessage.h"
#include "ACTable.h"

class EvalEngineKernel
{
public:
    std::set<std::string> m_subscriptions;
    std::set<std::string> m_publications;
    std::map<std::string, std::string> m_parameters;
    std::string m_identity;
    char *m_debug_stream_name;
    FILE *m_debug_stream;
    
    EvalEngineKernel();
    ~EvalEngineKernel();

    std::set<std::string> get_subscriptions();

    bool updateSubscriptions(std::string old_val, std::string new_val);

    std::set<std::string> get_publications();

    std::string generateRunWarning(std::string message);

    bool updatePublications(std::string old_val, std::string new_val);

    std::map<std::string, std::string> get_parameters();

    void set_debug_stream(char debug_stream_name[]);

    virtual bool SetParams(const std::map<std::string, std::string> &params){return false;};

    virtual bool UpdateVariable(const EEMessage &message){return false;};

    virtual bool Evaluate(std::vector<std::pair<std::string, std::string> > &updates, std::vector<std::string> &kernel_level_warnings){return false;};

    virtual bool AppCastingNotice(ACTable &actab){return false;};

    bool dbg_print(const char *format, ...);
};