
/* **************************************************************
  NAME: Raymond Turrisi 
  ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
  FILE: AgentInfo.h
  CIRC: November 2023
  DESC: 
    A serialization/deserialization class for 'Agent Information'
    which is related to convoying. Experimental and subject to change. 
    Should not be used outside the scope of ConvoyPD or ConvoySync.

  LICENSE:
    This is unreleased BETA code. No permission is granted or
    implied to use, copy, modify, and distribute this software 
    except by the author(s), or those designated by the author.
************************************************************** */
#include <string> 

class AgentInfo
{
public:
  std::string name; //0
  double x; //1
  double x_dot; //2
  double y; //3
  double y_dot; //4
  double z; //5
  double z_dot; //6
  double h; //7
  double h_dot; //8
  double u; //9
  double v; //10
  double utc; //11
  std::string color; //12
  double dist_err; //13
  double spd_err; //14

  AgentInfo();

  AgentInfo(std::string strrep);

  std::string repr(std::string delim=",");

};
