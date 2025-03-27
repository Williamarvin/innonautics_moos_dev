/************************************************************/
/*    NAME: Mikala Molina                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: EncounterReport.h                                       */
/*    DATE: June 26 2022                                   */
/************************************************************/

#ifndef EncounterReport_HEADER
#define EncounterReport_HEADER

#include <string>
#include <list>
#include "XYPoint.h"

class EncounterReport
{
   public:
  EncounterReport(std::string name="");
  ~EncounterReport() {}; 
 public: // Setters
  void setOS(std::string v) {m_os = v;}
  void setCN(std::string v) {m_cn = v;}
  void setRange(double v) {m_range = v;}
  void setRelBrg(double v) {m_rel_brg = v;}
  void setTargAng(double v) {m_targ_ang = v;}
  void setTime(double v) {m_time = v;}
  void setClassified(bool v) {m_classified = v;}

 public: // Getters
  std::string  getOS();
  std::string  getCN();
  double       getRange();
  double       getRelBrg();
  double       getTargAng();
  double       getTime();
  bool         getClassified();



 private:  
  std::string m_os;
  std::string m_cn;
  double  m_range;      
  double  m_rel_brg;
  double  m_targ_ang;
  double  m_time;
  bool    m_classified;
};

EncounterReport stringToEncounterReport(std::string);

#endif  
