
/* **************************************************************
  NAME: Raymond Turrisi 
  ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
  FILE: BHV_TaskFormationFollower.h
  CIRC: November 2023
  DESC: 
    Constructs a bid for the convoy ordering problem with a dubins
    turn. The total odometry of this proposed dubins turn / path
    is the bid. 

  LICENSE:
    This is unreleased BETA code. No permission is granted or
    implied to use, copy, modify, and distribute this software 
    except by the author(s), or those designated by the author.
************************************************************** */
 
#ifndef BHV_TASK_CONVOY_HEADER
#define BHV_TASK_CONVOY_HEADER

#include <string>
#include <vector>
#include "IvPTaskBehavior.h"
#include "dubin.h"

class IvPDomain;
class BHV_TaskFormationFollower : public IvPTaskBehavior {
public:
  BHV_TaskFormationFollower(IvPDomain);
  ~BHV_TaskFormationFollower() {}

  // virtuals defined
  void   onHelmStart();
  double getTaskBid();
  bool   setParam(std::string, std::string);

  std::vector<VarDataPair> applyFlagMacros(std::vector<VarDataPair>);

  void         onIdleState();
  IvPFunction* onRunState();

 protected:  // Configuration Parameters

 protected:  // State Variables
 double m_turn_radius;
 bool m_turn_radius_set;
 int m_pos_idx;

};

#ifdef WIN32
   // Windows needs to explicitly specify functions to export from a dll
   #define IVP_EXPORT_FUNCTION __declspec(dllexport) 
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_TaskFormationFollower(domain);}
}
#endif






