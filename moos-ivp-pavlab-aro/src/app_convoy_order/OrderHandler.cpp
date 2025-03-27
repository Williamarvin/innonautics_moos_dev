/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: OrderHandler.cpp                                     */
/*    DATE: June 4th, 2022                                       */
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
#include "OrderHandler.h"
#include "FileBuffer.h"

using namespace std;


//--------------------------------------------------------
// Constructor

OrderHandler::OrderHandler()
{
  m_verbose  = false;
}

//--------------------------------------------------------
// Procedure: setParam()

bool OrderHandler::setParam(string param, string value)
{
  if(param == "verbose")
    return(setBooleanOnString(m_verbose, value));
  if(param == "file")
    return(setNonWhiteVarOnString(m_file, value));
  
  return(false);
}


//--------------------------------------------------------
// Procedure: handle()

bool OrderHandler::handle()
{
  if(m_file == "") {
    cout << "Please specify a file name. Exiting." << endl;
    return(false);
  }

  // ==============================================
  // Part 1: Parse the file
  // ==============================================
  vector<string> lines = fileBuffer(m_file);
  if(lines.size() == 0) {
    cout << "File not found or empty:" << m_file << endl;
    return(false);
  }

  for(unsigned int i=0; i<lines.size(); i++) {
    string orig = stripBlankEnds(lines[i]);
    string line = orig;
    if(line.size() == 0)
      continue;
    if(strBegins(line, "//") || strBegins(line, "#"))
      continue;
    
    bool result = true;
    if(strBegins(line, "remove:") || strBegins(line, "rem:")) {
      string vname = rbiteStringX(line, ':');
      result = m_odetector.removeVehicle(vname);
    }    
    else
      result = m_odetector.addPairing(line);

    if(!result) {
      cout << "Unhandled line:" << line << endl;
      return(false);
    }

    if(m_verbose) {
      m_odetector.findConvoy();
      cout << "line: " << orig << endl;
      cout << "      " << m_odetector.getConvoySummary() << endl;
    }
  }

  m_odetector.findConvoy();
  cout << "FINAL: " << m_odetector.getConvoySummary() << endl;
  return(true);
}

