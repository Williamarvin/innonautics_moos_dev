#pragma once
#include <string> 
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string> 

class EEMessage {
  public:  
    std::string name;
    std::string contents_str;
    double contents_dbl;
    std::string src;
    std::string aux;
    double timestamp;
    std::string dtype;
    std::string msg_type;
    std::string src_community;
    
  public:

    //Include better options for maintaining a time-type message
    EEMessage();

    EEMessage(std::string varname, std::string message, double time_stamp);

    EEMessage(std::string varname, double message, double time_stamp);

    EEMessage(CMOOSMsg moos_msg);
    
    //TODO: Add functions for getting the contents for each of the state variables
    std::string getVarName() const;

    std::string msg() const;

    double time_stamp() const;
};