/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionInput.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "OpinionInput.h"
#include "XYFormatUtilsRangePulse.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

OpinionInput::OpinionInput()
{
  m_danger_time = 0;
  m_danger_pulse_count = 0;
  m_danger_max  = 100;
  m_danger_min  = 40;
  m_danger_min_range = 1;

  m_nav_x = -1;
  m_nav_y = -1;
  m_odometry = 0.0;

  m_send_new_ignore_set = false;

  m_min_batt = 11.5;
  m_max_batt = 15.5;
  m_m300_batt_voltage = 0;
  m_got_real_batt_voltage = false;
  m_current_batt_voltage = 0.0; 
  
}

//---------------------------------------------------------
// Destructor

OpinionInput::~OpinionInput()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool OpinionInput::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if(key == "DANGER_PULSE"){
       m_danger_time = msg.GetTime();
       XYRangePulse new_pulse = stringStandard2RangePulse(msg.GetString());
       std::string label = toupper( new_pulse.get_label() );
       if (label.find("WARN") != std::string::npos) {
	 m_danger_pulse = new_pulse;
	 m_danger_pulse_count ++;
       }

    } else if (key == "NAV_X") {
      m_nav_x = msg.GetDouble();
      m_nav_x_vec.push_back( msg.GetDouble() ); 

    } else if (key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
      m_nav_y_vec.push_back( msg.GetDouble() );

    } else if (key == "CONTACTS_LIST") {
        // parse message
      std::vector<std::string> svector = parseString(msg.GetString(), ',');
      for (unsigned int i=0; i<svector.size(); i++) {
	m_contacts.insert(svector[i]);
      }
      m_send_new_ignore_set = true;

    } else if (key == "COVERAGE_LIST") {
      // Make a new set to erase the old set.
      std::set<std::string> new_voronoi_active_set;
        // parse message
      std::vector<std::string> svector = parseString(msg.GetString(), ',');
      for (unsigned int i=0; i<svector.size(); i++) {
	new_voronoi_active_set.insert(svector[i]);
      }

      m_voronoi_active = new_voronoi_active_set;
      m_send_new_ignore_set = true; 

    } else if (key == "M300_BATT_VOLTAGE") {
      m_m300_batt_voltage = msg.GetDouble();
      m_got_real_batt_voltage = true; 

    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool OpinionInput::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool OpinionInput::Iterate()
{
  AppCastingMOOSApp::Iterate();

  calculateInputs(MOOSTime());
  publishInputs();

  if ( m_send_new_ignore_set) {
    publishIgnoreLists(); 
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool OpinionInput::OnStartUp()
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
    if(param == "danger_max") {
      handled = setDoubleOnString(m_danger_max, value); 
    }
    else if(param == "danger_min") {
      handled = setPosDoubleOnString(m_danger_min, value);
    }
    else if(param == "danger_min_range") {
      handled = setPosDoubleOnString(m_danger_min_range, value);
    }


    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void OpinionInput::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("DANGER_PULSE", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("CONTACTS_LIST", 0);
  Register("COVERAGE_LIST", 0);
  Register("M300_BATT_VOLTAGE",0); 
  
}


//------------------------------------------------------------
// Procedure: buildReport()

bool OpinionInput::buildReport() 
{


  ACTable actab(2);
  actab << " Danger Pulses | Last spec ";
  actab.addHeaderLines();
  actab << uintToString(m_danger_pulse_count) << m_danger_pulse.get_spec();
  m_msgs << actab.getFormattedString();
  m_msgs << endl;
  m_msgs << "                                                           " << endl;
  
  ACTable actab2(2);
  actab2 << " Odometry | Batt Volts ";
  actab2.addHeaderLines();
  actab2 << doubleToString(m_odometry) << doubleToString(m_current_batt_voltage); 
  m_msgs << actab2.getFormattedString();
  m_msgs << endl;

  
  m_msgs << "                                                           " << endl;
  m_msgs << "=  Input Values  ==========================================" << endl;
  m_msgs << "===========================================================" << endl;
  std::map<std::string, double>::iterator it;
  for (it = m_opinion_inputs.begin(); it!= m_opinion_inputs.end(); it++) {
    m_msgs << "  " << it->first << ": " <<  doubleToStringX(it->second, 4) << endl; 
  }
  

  return(true);
}


//----------------------------------------------------------
// Procedure: calculateInputs(time)

bool OpinionInput::calculateInputs(double time)
{

  // Danger opinion input. 
  double dist_to_center = distPointToPoint(m_nav_x, m_nav_y, m_danger_pulse.get_x(), m_danger_pulse.get_y());

  double danger_pulse_rad = (time - m_danger_time) / m_danger_pulse.get_duration() * m_danger_pulse.get_radius(); 
  // Check if conditions are met
  bool stale = ( time - m_danger_time) > m_danger_pulse.get_duration();
  bool close = (dist_to_center < danger_pulse_rad);
  bool recieved_one = m_danger_pulse_count > 0;

  if (!stale && close && recieved_one) {
    m_opinion_inputs["DANGER_LEVEL"] = (m_danger_max - m_danger_min) * (m_danger_pulse.get_radius() - dist_to_center) / m_danger_pulse.get_radius() + m_danger_min;
    m_opinion_inputs["DANGER_LEVEL"] = m_danger_max;
    
  } else {
    m_opinion_inputs["DANGER_LEVEL"] = m_danger_min; 
  }

  // Coverage input

  // update odometry
  
  bool same_length = m_nav_x_vec.size() == m_nav_y_vec.size(); 
  if ( (m_nav_x_vec.size() >= 2) && same_length) {
    //
    unsigned int i;
    for (i = 0; i<m_nav_x_vec.size()-1; i++) {
      double dx = m_nav_x_vec[i+1] - m_nav_x_vec[i];
      double dy = m_nav_y_vec[i+1] - m_nav_y_vec[i];
      m_odometry += sqrt( dx * dx + dy * dy);
    }
    
    double last_x = m_nav_x_vec[m_nav_x_vec.size()-1];
    double last_y = m_nav_y_vec[m_nav_y_vec.size()-1];
    m_nav_x_vec.clear();
    m_nav_y_vec.clear();
    m_nav_x_vec.push_back(last_x);
    m_nav_y_vec.push_back(last_y);
    
  }
  

  double max_heron_range = 1 * 60 * 60 * 2;
  if (m_got_real_batt_voltage) {
    m_current_batt_voltage = m_m300_batt_voltage;
  } else {
    m_current_batt_voltage = (m_min_batt - m_max_batt) * (m_odometry / max_heron_range) + m_max_batt;
  }


  double max_battery_bias = 50.0;
  double battery_exhausted = (m_max_batt - m_current_batt_voltage)/ (m_max_batt - m_min_batt);
  //double a = - max_battery_bias / (m_max_batt - m_min_batt);
  //double b = max_battery_bias;
  
  
  m_opinion_inputs["COVERAGE_INPUT"] = 50.0 * battery_exhausted * battery_exhausted + 50.0; 
  
  return(true); 
}



//----------------------------------------------------------
// Procedure: calculateInputs(time)

bool OpinionInput::publishInputs()
{

  std::map<std::string, double>::iterator it;
  for (it = m_opinion_inputs.begin(); it!= m_opinion_inputs.end(); it++) {
    Notify(it->first, it->second); 
  }
  
  return(true); 
}




//---------------------------------------------------------------
// publishIgnoreLists()
//         Any and all
bool OpinionInput::publishIgnoreLists()
{

  std::string val = "";
  std::set<std::string>::iterator it;
  for (it = m_contacts.begin(); it != m_contacts.end(); it++){
    if (m_voronoi_active.count(*it)==0) {
      val += *it + ",";
    }
  }

  // remove the last comma if needed
  if (val.length()>0)
    val = val.substr(0, val.size()-1);

  Notify("PROX_SET_IGNORE_LIST", val);
  m_send_new_ignore_set = false;
  
  
  return(true);
}
