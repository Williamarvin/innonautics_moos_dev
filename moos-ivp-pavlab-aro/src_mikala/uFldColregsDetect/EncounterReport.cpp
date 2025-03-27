/************************************************************/
/*    NAME: Mikala Molina                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: EncounterReport.cpp                                     */
/*    DATE: June 26 2023                                   */
/************************************************************/

#include <vector>
#include <iterator>
#include "MBUtils.h"
#include "EncounterReport.h"
#include "XYPoint.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

EncounterReport::EncounterReport(std::string name)
{
  m_cn = "none";
  m_os = "none";
  m_range = 0;
  m_rel_brg = 0;
  m_targ_ang = 0;
  m_time = 0;
  m_classified = false;


}

//---------------------------------------------------------
//Procedure: stringToNodeReport()

EncounterReport stringToEncounterReport(std::string str)
{
  EncounterReport empty_encounter_report; 
  EncounterReport encounter_report;

  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(biteStringX(svector[i], '='));
    string value = svector[i];
    string unhandled_contents = ""; 
    double dval  = atof(value.c_str());

      if(param == "OS" || param == "os")
        encounter_report.setOS(value);
      else if(param == "CN" || param == "cn")
        encounter_report.setCN(value);
      else if(param == "RNG" || param == "rng")
        encounter_report.setRange(dval);
      else if(param == "REL_BRG" || param == "rel_brg")
        encounter_report.setRelBrg(dval);
      else if(param == "TARG_ANG" || param == "targ_ang")
        encounter_report.setTargAng(dval);
      else if(param == "TIME" || param == "time")
        encounter_report.setTime(dval);
      else
       unhandled_contents += svector[i] + ", ";
      
  }
  return(encounter_report);
}
//---------------------------------------------------------

// Procedure: getOS() 
std::string EncounterReport::getOS()
{
  return(m_os);
}
//---------------------------------------------------------
// Procedure: getCN()
std::string EncounterReport::getCN()
{
  return(m_cn);
}
//---------------------------------------------------------
// Procedure: getRange()
double EncounterReport::getRange()
{
  return(m_range);
}
//---------------------------------------------------------
// Procedure: getRelBrg()
double EncounterReport::getRelBrg()
{
  return(m_rel_brg);
}
//---------------------------------------------------------
// Procedure: getTargAng()
double EncounterReport::getTargAng()
{
  return(m_targ_ang);
}
//---------------------------------------------------------
// Procedure: getTime()
double EncounterReport::getTime()
{
  return(m_time);
}
//---------------------------------------------------------
// Procedure: getClassified()
bool EncounterReport::getClassified()
{
  return(m_classified);
}
