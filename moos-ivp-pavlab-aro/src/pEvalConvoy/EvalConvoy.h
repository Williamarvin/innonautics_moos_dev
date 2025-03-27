/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: EvalConvoy.h                                         */
/*    DATE: July 11th, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef EVAL_CONVOY_HEADER
#define EVAL_CONVOY_HEADER

#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "EvalConvoyEngine.h"

class EvalConvoy : public AppCastingMOOSApp
{
 public:
  EvalConvoy() {};
  ~EvalConvoy() {};

 protected: // Standard MOOSApp functions to overload  
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: 
  bool buildReport();
  void registerVariables();

  void shareStatRecap();
  bool handleMailNodeReport(std::string str, std::string& whynot);
  
 private: // State variables

  EvalConvoyEngine m_engine;

  std::string m_group_name;

  // If match_group is true only send RECAP_ALLY messages to
  // group members.
  bool        m_match_group; 
  
  double      m_last_share_tstamp;
  std::string m_last_stat_recap;
};

#endif 


