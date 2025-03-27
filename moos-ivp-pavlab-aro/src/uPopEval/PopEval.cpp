/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PopEval.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "PopEval.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

PopEval::PopEval()
{
  m_stale_thresh = 10.0;

  m_odom_stat_report = "";
  m_batt_stat_report = "";
  
}

//---------------------------------------------------------
// Destructor

PopEval::~PopEval()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool PopEval::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if(key == "NODE_STATUS_MSG") {
      if (!handledNodeStatusMsg(msg.GetString()))
	reportRunWarning("Unhandled Mail: " + key);

    }  else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool PopEval::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool PopEval::Iterate()
{
  AppCastingMOOSApp::Iterate();
  calculateStats();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool PopEval::OnStartUp()
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
    else if (param == "stale_thresh") {
      handled = setPosDoubleOnString(m_stale_thresh, value);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void PopEval::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_STATUS_MSG", 0);
}


//---------------------------------------------------------
// Procedure: handledNodeStatusMsg(std::string msg)
bool PopEval::handledNodeStatusMsg(std::string msg)
{
  std::string orig_msg = msg; 
  std::string node_name = "";
  double node_odom;
  double node_batt;

  bool got_name = false;
  bool got_odom = false;
  bool got_batt = false;

  vector<string> svector = parseString(msg, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    if((param == "name") && (value != "")){
      node_name = value;
      got_name = true;
      
    } else if(param == "odom") {
      got_odom = setDoubleOnString(node_odom, value);
 
    } else if(param == "batt") {
      got_batt = setDoubleOnString(node_batt, value);

    }
  }

  if (!got_name || !got_odom || !got_batt )
    return(false);

  m_odom_vals[node_name] = node_odom;
  m_batt_vals[node_name] = node_batt;
  m_times[node_name] = MOOSTime();

  m_raw_msgs[node_name] = orig_msg;
  
  
   return(true); 
  
}


//---------------------------------------------------------
// Procedure: calculateStats()
void PopEval::calculateStats()
{

  double odom_run_ave = 0.0;
  double batt_run_ave = 0.0;
  double count = 1.0;

  double odom_max = std::numeric_limits<double>::min();
  double odom_min = std::numeric_limits<double>::max();
  double batt_max = std::numeric_limits<double>::min();
  double batt_min = std::numeric_limits<double>::max();
  
  
  std::map<std::string, double>::iterator it;
  for (it = m_odom_vals.begin(); it != m_odom_vals.end(); it++){
    std::string name = it->first;

    if (m_batt_vals.count(name) == 0)
      continue;
    if (m_times.count(name) == 0)
      continue;
    
    double odom = it->second;
    double batt = m_batt_vals[name]; 
    double time = m_times[name];

    if ( (MOOSTime() - time) > m_stale_thresh)
      continue;

    // update running averages
    odom_run_ave = ((odom_run_ave * (count - 1.0) ) + odom) / (count);
    batt_run_ave = ((batt_run_ave * (count - 1.0) ) + batt) / (count);

    // update min and maxes
    if (odom > odom_max)
      odom_max = odom;
    
    if (odom < odom_min)
      odom_min = odom;

    if (batt > batt_max)
      batt_max = batt;

    if (batt < batt_min)
      batt_min = batt; 

    count = count + 1.0; 
    
  }


  // build reports
  m_odom_stat_report = "Odom:  Min = " + doubleToStringX(odom_min, 2);
  m_odom_stat_report += ", Ave = " + doubleToStringX(odom_run_ave, 2);
  m_odom_stat_report += ", Max = " + doubleToStringX(odom_max, 2);

  m_batt_stat_report = "Batt:  Min = " + doubleToStringX(batt_min, 2);
  m_batt_stat_report += ", Ave = " + doubleToStringX(batt_run_ave, 2);
  m_batt_stat_report += ", Max = " + doubleToStringX(batt_max, 2);

  Notify("ODOM_STAT_REPORT", m_odom_stat_report);
  Notify("BATT_STAT_REPORT", m_batt_stat_report);
  

}





  
//------------------------------------------------------------
// Procedure: buildReport()

bool PopEval::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << " Population Statistics:                     " << endl;
  m_msgs << " * " << m_odom_stat_report << endl;
  m_msgs << " * " << m_batt_stat_report << endl;

  m_msgs << "                                            " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << " Node Stat Msgs:                            " << endl;

  std::map<std::string, std::string>::iterator it;
  for (it = m_raw_msgs.begin(); it != m_raw_msgs.end(); it++){
    m_msgs << " -> " << it->second;
    double time_delay = MOOSTime() - m_times[it->first]; 
    m_msgs << "  (" << doubleToStringX(time_delay,2) << "s ago)" << endl;
  }

 

  return(true);
}




