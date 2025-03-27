/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionAnalysis.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "OpinionAnalysis.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

OpinionAnalysis::OpinionAnalysis()
{
  m_last_time_option_rec = MOOSTime();
  m_last_option_name = "";
}

//---------------------------------------------------------
// Destructor

OpinionAnalysis::~OpinionAnalysis()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool OpinionAnalysis::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "OPTION") 
       handleOption(msg.GetString()); 

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool OpinionAnalysis::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool OpinionAnalysis::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Update periodically..
  if (m_got_first_option) {
    handleOption(m_last_option_name);

    // Post Option times
    std::map<std::string,double>::iterator it;
    for (it = m_option_times.begin(); it != m_option_times.end(); it++){
      Notify("OPINION_TIME_" + it->first, it->second);
    }
  }
    
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool OpinionAnalysis::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void OpinionAnalysis::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
   Register("OPTION", 0);
}


//---------------------------------------------------------
// Procedure: handleOption()

void OpinionAnalysis::handleOption(std::string val)
{
  std::string option_name = toupper(val);

  // how much time has elapsed since the last option was
  // recorded
  double time_elapsed = MOOSTime() - m_last_time_option_rec;


  // don't update the first time
  if (!m_got_first_option){
    // The time has elapsed under the last option recieved
    if (m_option_times.count(option_name) == 0){
      // this is the first time we've recorded this option
      m_option_times[m_last_option_name] = time_elapsed;
    } else {
      m_option_times[m_last_option_name] = time_elapsed + m_option_times[m_last_option_name];
    }
  } else
    m_got_first_option = true;

  // update with the next option name (sometimes this
  // is the new option).
  m_last_option_name = option_name;
  m_last_time_option_rec = MOOSTime();
  
  
  return;
}



//------------------------------------------------------------
// Procedure: buildReport()

bool OpinionAnalysis::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




