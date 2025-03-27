/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: LogEvalHandler.cpp                                   */
/*    DATE: August 24th, 2021                                    */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include "MBUtils.h"
#include "LogEvalHandler.h"
#include "LogUtils.h"

using namespace std;


//--------------------------------------------------------
// Constructor

LogEvalHandler::LogEvalHandler()
{
  m_file_in  = 0;
  m_verbose  = false;
}

//--------------------------------------------------------
// Procedure: setALogFile()

bool LogEvalHandler::setALogFile(string alogfile)
{
  m_file_in = fopen(alogfile.c_str(), "r");
  if(!m_file_in) {
    cout << "input file not found or unable to open - exiting" << endl;
    return(false);
  }
  
  return(true);
}

//--------------------------------------------------------
// Procedure: setParam()

bool LogEvalHandler::setParam(string param, string value)
{
  if(param == "on_tail_thresh")
    return(m_engine.setParam(param, value));
  if(param == "alignment_thresh")
    return(m_engine.setParam(param, value));
  if(param == "verbose")
    return(setBooleanOnString(m_verbose, value));
  if(param == "report")
    return(addReportType(value));
  if(param == "track_err_snap")
    return(m_engine.setParam(param, value));
  
  return(false);
}


//--------------------------------------------------------
// Procedure: addReportType()

bool LogEvalHandler::addReportType(string report_type)
{
  if(vectorContains(m_reports, report_type))
    return(false);
  
  if(report_type == "appcast")
    m_reports.push_back(report_type);  
  else if(report_type == "track_err_bins")
    m_reports.push_back(report_type);
  else
    return(false);
  
  return(true);
}

//--------------------------------------------------------
// Procedure: handle()

bool LogEvalHandler::handle()
{
  if(!m_file_in)
    return(false);

  // ==============================================
  // Part 1: Parse and handle the alog file
  // ==============================================
  bool done = false;
  while(!done) {
    ALogEntry entry = getNextRawALogEntry(m_file_in, true);

    // Check if the line is a comment
    if(entry.getStatus() == "invalid")
      continue;
    // Check for end of file
    if(entry.getStatus() == "eof")
      break;
    
    string varname = entry.getVarName();

    bool handled = true;
    if(varname == "CONVOY_RECAP") {
      double tstamp = entry.getTimeStamp();
      m_engine.setCurrTime(tstamp);
      handled = m_engine.handleRecap(entry.getStringVal());
      m_engine.updateMetrics();
    }
    else if(varname == "CONVOY_STAT_RECAP")
      handled = m_engine.handleStatRecap(entry.getStringVal());
    else if(varname == "CONVOY_SPD_POLICY")
      handled = m_engine.handleSpdPolicy(entry.getStringVal());

    if(!handled) {
      m_unhandled_lines++;
      cout << "Bad line: [" << entry.getRawLine() << "]" << endl;
    }
  }

  if(m_file_in)
    fclose(m_file_in);

  return(true);
}

//--------------------------------------------------------
// Procedure: makeReports()

void LogEvalHandler::makeReports()
{
  if(m_reports.size() == 0) {
    cout << "No requested reports." << endl;
    return;
  }
    
  for(unsigned int i=0; i<m_reports.size(); i++)
    makeReport(m_reports[i]);
}


//--------------------------------------------------------
// Procedure: makeReport()

void LogEvalHandler::makeReport(string report_type)
{
  if(report_type == "appcast") {
    vector<string> svector = m_engine.buildReport();
    for(unsigned int i=0; i<svector.size(); i++)
      cout << svector[i] << endl;
  }

  if(report_type == "track_err_bins") {
    vector<string> svector = m_engine.getRepTrackErrBins();
    for(unsigned int i=0; i<svector.size(); i++)
      cout << svector[i] << endl;
  }
}


