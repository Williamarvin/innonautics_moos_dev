
/* **************************************************************
  NAME: Raymond Turrisi
  ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
  FILE: AgentInfo.cpp
  CIRC: November 2023

  LICENSE:
    This is unreleased BETA code. No permission is granted or
    implied to use, copy, modify, and distribute this software
    except by the author(s), or those designated by the author.
************************************************************** */

#include <string>
#include <vector>
#include "MBUtils.h"
#include "AgentStateInfo.h"

using namespace std;

arma::vec::fixed<6> convertToVecFixed6(const std::string& str)
{
    std::istringstream iss(str);
    arma::vec::fixed<6> result;
    double value;
    size_t i = 0;
    while (iss >> value && i < 6)
    {
        result(i++) = value;
    }
    return result;
}

// Helper function to convert an Armadillo vector to a string
std::string armaVecToString(const arma::vec::fixed<6> &vec)
{
    std::ostringstream out;
    vec.print(out);
    std::string result = out.str();
    // Remove the new line characters and the brackets
    result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
    result.erase(std::remove(result.begin(), result.end(), '['), result.end());
    result.erase(std::remove(result.begin(), result.end(), ']'), result.end());
    return result;
}

AgentStateInfo::AgentStateInfo()
{
    // Information about the agent
    name = "larry";
    time_se = 0;
    color = "yellow";

    // Initialize vectors to zero
    q.zeros();
    q_dot.zeros();
    q_dotdot.zeros();
    r_err = 0;
    r_reference = 0;
    q_err.zeros();
    q_dot_err.zeros();
    q_dotdot_err.zeros();
    nu.zeros();
    nu_dot.zeros();
    nu_err.zeros();
    nu_dot_err.zeros();
}

AgentStateInfo::AgentStateInfo(std::string strrep)
{
    std::vector<std::string> fields = parseString(strrep, ',');

    for (auto field : fields)
    {
        std::string key = biteString(field, '=');
        std::string value = field;
        std::cerr << "key: " << key << " value: " << value << std::endl;
        if (key == "name")
            name = value;
        else if (key == "time_se")
            time_se = std::stod(value);
        else if (key == "color")
            color = value;
        else if (key == "id")
            id = std::stoi(value);
        else if (key == "q")
            q = convertToVecFixed6(value);
        else if (key == "q_dot")
            q_dot = convertToVecFixed6(value);
        else if (key == "q_dotdot")
            q_dotdot = convertToVecFixed6(value);
        else if (key == "q_err")
            q_err = convertToVecFixed6(value);
        else if (key == "q_dot_err")
            q_dot_err = convertToVecFixed6(value);
        else if (key == "q_dotdot_err")
            q_dotdot_err = convertToVecFixed6(value);
        else if (key == "nu")
            nu = convertToVecFixed6(value);
        else if (key == "nu_dot")
            nu_dot = convertToVecFixed6(value);
        else if (key == "nu_err")
            nu_err = convertToVecFixed6(value);
        else if (key == "nu_dot_err")
            nu_dot_err = convertToVecFixed6(value);
        else if (key == "r_err")
            r_err = std::stod(value);
        else if (key == "r_reference")
            r_reference = std::stod(value);
    }
}

AgentStateInfo::AgentStateInfo(const AgentStateInfo &asi) {
    name = asi.name;
    time_se = asi.time_se;
    color = asi.color;
    id = asi.id;
    q = asi.q;
    q_dot = asi.q_dot;
    q_dotdot = asi.q_dotdot;
    r_err = asi.r_err;
    r_reference = asi.r_reference;
    q_err = asi.q_err;
    q_dot_err = asi.q_dot_err;
    q_dotdot_err = asi.q_dotdot_err;
    nu = asi.nu;
    nu_dot = asi.nu_dot;
    nu_err = asi.nu_err;
    nu_dot_err = asi.nu_dot_err;
}


std::string AgentStateInfo::repr(std::string delim)
{
    std::string result;
    result = "name=" + name + delim +
             "time_se=" + std::to_string(time_se) + delim +
             "color=" + color + delim +
             "id=" + std::to_string(id) + delim +
             "q=" + armaVecToString(q) + delim +
             "q_dot=" + armaVecToString(q_dot) + delim +
             "q_dotdot=" + armaVecToString(q_dotdot) + delim +
             "q_err=" + armaVecToString(q_err) + delim +
             "q_dot_err=" + armaVecToString(q_dot_err) + delim +
             "q_dotdot_err=" + armaVecToString(q_dotdot_err) + delim +
             "nu=" + armaVecToString(nu) + delim +
             "nu_dot=" + armaVecToString(nu_dot) + delim +
             "nu_err=" + armaVecToString(nu_err) + delim +
             "nu_dot_err=" + armaVecToString(nu_dot_err) + delim +
             "r_err=" + std::to_string(r_err) + delim +
             "r_reference=" + std::to_string(r_reference);
    return result;
}
