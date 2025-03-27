#include "EvalEngineMessage.h"

//TODO: Consider removing EEMessage entirely and using CMOOSMsg
// Include better options for maintaining a time-type message
EEMessage::EEMessage()
{
    name = "NULL";
    contents_str = "NULL";
    contents_dbl = -1;
    timestamp = -1;
    aux = "NULL";
    dtype = "NDEF";
    msg_type = "NDEF";
    src = "NDEF";
    src_community = "NDEF";
}

EEMessage::EEMessage(std::string varname, std::string message, double time_stamp)
{
    name = varname;
    contents_str = message;
    contents_dbl = -1;
    timestamp = time_stamp;
    aux = "NULL";
    dtype = "NDEF";
    msg_type = "NDEF";
    src = "NDEF";
    src_community = "NDEF";
}

EEMessage::EEMessage(std::string varname, double message, double time_stamp)
{
    name = varname;
    contents_str = std::to_string(message);
    contents_dbl = message;
    timestamp = time_stamp;
    aux = "NULL";
    dtype = "NDEF";
    msg_type = "NDEF";
    src = "NDEF";
    src_community = "NDEF";
}

EEMessage::EEMessage(CMOOSMsg moos_msg)
{
    // Reference:
    name = moos_msg.GetKey();

    if (moos_msg.IsString())
    {
        contents_str = moos_msg.GetAsString();
        dtype = "STRING";
    }
    else if (moos_msg.IsDouble())
    {
        // contents_str = std::string(moos_msg.GetDouble());
        contents_str = moos_msg.GetAsString();
        contents_dbl = moos_msg.GetDouble();
        dtype = "DOUBLE";
    }
    else if (moos_msg.IsBinary())
    {
        // TODO: Accomodate binary messages?
        dtype = "BINARY";
    }
    else
    {

        // TODO: Include further error handling?
    }
    src = moos_msg.GetSource();
    // TODO: Accomodate auxiliary?
    aux = "NULL";
    timestamp = moos_msg.GetTime();
    msg_type = moos_msg.GetSource();
    src_community = moos_msg.GetCommunity();
}

// TODO: Add functions for getting the contents for each of the state variables
std::string EEMessage::getVarName() const
{
    return name;
}

std::string EEMessage::msg() const
{
    return contents_str;
}

double EEMessage::time_stamp() const
{
    return timestamp;
}