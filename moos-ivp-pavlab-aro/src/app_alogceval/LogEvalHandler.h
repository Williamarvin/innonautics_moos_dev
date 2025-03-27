/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: LogEvalHandler.h                                     */
/*    DATE: August 24th, 2021                                    */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef LOG_EVAL_HANDLER_HEADER
#define LOG_EVAL_HANDLER_HEADER

#include <string>
#include <map>
#include "EvalConvoyEngine.h"

class LogEvalHandler
{
 public:
  LogEvalHandler();
  ~LogEvalHandler() {}

  bool handle();
  void makeReports();
  
  bool setALogFile(std::string);
  bool setParam(std::string, std::string);
  bool addReportType(std::string);
  
 protected: 
  void makeReport(std::string report_type);

protected: // Config vars

  FILE *m_file_in;
  bool   m_verbose;

  std::vector<std::string> m_reports;
  
 protected: // State vars

  unsigned int m_unhandled_lines;
  
  EvalConvoyEngine m_engine;
};

#endif


