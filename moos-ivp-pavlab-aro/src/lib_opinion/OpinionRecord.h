/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionRecord.h                                 */
/*    DATE: April 5th, 2023                                 */
/************************************************************/

#ifndef OpinionRecord_HEADER
#define OpinionRecord_HEADER

#include <string>
#include <vector>
#include "MBUtils.h"
#include <map>


class OpinionRecord
{
 public:
  OpinionRecord();
  ~OpinionRecord();

  bool setRecordFromMsg(std::string msg, double msg_time);

  std::string getVname() {return(m_vname);};
  std::string getGroup() {return(m_group);};
  double getTime() {return(m_record_time);};

  bool isOptionExist(std::string q_val);
  bool getOpinionForOption(std::string q_val, double& val);
  std::string getLastRecordMsg() {return(m_last_record_msg);};

  double getOpinionsSquared();
  std::string getStrongestOpinion();

  int getNumberOfOpinions() {return(m_opinion_vals.size());};
  

  
 private:

  std::string m_last_record_msg; 
 
  std::string m_vname;  // might be redundant
  std::string m_group;
  std::map<std::string, double> m_opinion_vals;
  double m_record_time; 

  
};

#endif
