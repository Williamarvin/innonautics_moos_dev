
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
#include <MBUtils.h>
#include "AgentInfo.h"

using namespace std;

AgentInfo::AgentInfo()
{
    name = "LARRY!";
    x = 0;
    x_dot = 0;
    y = 0;
    y_dot = 0;
    z = 0;
    z_dot = 0;
    h = 0;
    h_dot = 0;
    u = 0;
    v = 0;
    utc = 0;
    color = "yellow";
}

AgentInfo::AgentInfo(std::string strrep)
{
    std::vector<std::string> fields;

    fields = parseString(strrep, ',');

    for (int i = 0; i < fields.size(); i++)
    {
        biteString(fields[i], '=');
        switch (i)
        {
        case 0:
            name = fields[i];
            break;
        case 1:
            x = stod(fields[i]);
            break;
        case 2:
            x_dot = stod(fields[i]);
            break;
        case 3:
            y = stod(fields[i]);
            break;
        case 4:
            y_dot = stod(fields[i]);
            break;
        case 5:
            z = stod(fields[i]);
            break;
        case 6:
            z_dot = stod(fields[i]);
            break;
        case 7:
            h = stod(fields[i]);
            break;
        case 8:
            h_dot = stod(fields[i]);
            break;
        case 9:
            u = stod(fields[i]);
            break;
        case 10:
            v = stod(fields[i]);
            break;
        case 11:
            utc = stod(fields[i]);
            break;
        case 12:
            color = fields[i];
            break;
        case 13:
            dist_err = stod(fields[i]);
            break;
        case 14:
            spd_err = stod(fields[i]);
            break;
        default:
            break;
        }
    }
}

std::string AgentInfo::repr(std::string delim)
{
    std::string result;
    result = string("name=") + name + delim + 
    string("x=") + floatToString(x, 2) + delim + 
    string("x_dot=") + floatToString(x_dot, 2) + delim + 
    string("y=") + floatToString(y, 2) + delim + 
    string("y_dot=") + floatToString(y_dot, 2) + delim + 
    string("z=") + floatToString(z, 2) + delim + 
    string("z_dot=") + floatToString(z_dot, 2) + delim + 
    string("h=") + floatToString(h, 2) + delim + 
    string("h_dot=") + floatToString(h_dot, 2) + delim + 
    string("u=") + floatToString(u, 2) + delim + 
    string("v=") + floatToString(v, 2) + delim + 
    string("utc=") + floatToString(utc, 3) + delim + 
    string("color=") + color + delim + 
    string("dist_err=") + floatToString(dist_err,3) + delim + 
    string("spd_err=") + floatToString(spd_err,3) + delim;
    return result;
}
