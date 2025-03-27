/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: FldEvalOption.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "FldEvalOption.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

FldEvalOption::FldEvalOption()
{
  m_text_x = 0.0;
  m_text_y = 0.0;
  m_text_font_size = 10;
}

//---------------------------------------------------------
// Destructor

FldEvalOption::~FldEvalOption()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool FldEvalOption::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    string sval = msg.GetString();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();

a    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    bool handled = false;
    if ((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL"))
    {
      handled = handleMailNodeReport(sval);
    }
    else if (m_vehicles_registered_for_option.count(key) > 0)
    {
      // get the vehicle name by removing the "OPTION_"
      std::string vname;

      if (key.size() > 7)
      {
        vname = tolower(key.erase(0, 7));

        m_vehicle_opt_status[vname] = toupper(sval);
        handled = true;
      }
    }
    else if (m_registered_input_vars_plus_veh.count(key) > 0)
    {
      handled = handleInputVarMsg(key, msg.GetDouble());
    }
    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);

    if (!handled)
      reportRunWarning("Unhandled Mail: " + key);
    else
      retractRunWarning("Unhandled Mail: " + key);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool FldEvalOption::OnConnectToServer()
{
  // registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool FldEvalOption::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Now build out the report
  // should look like this:
  // Option 1: vname1, vname2,
  //           vname6, vname7
  //           Utility 1:
  //           Utility 2:
  //           Utility 3:
  // Option 2: vname1, vname_2
  //           Utility 1:
  //           Utility 2:
  //           Utility 3:
  //        ''       ''
  // Option N:

  // The report will be stored in this vector of strings:
  // in m_report
  m_report.clear();

  // Step 1.  Find all the active options:
  std::set<std::string> active_options;
  std::map<std::string, std::set<std::string> > vehicle_assignment;

  std::map<std::string, std::string>::iterator it;
  for (it = m_vehicle_opt_status.begin(); it != m_vehicle_opt_status.end(); it++)
  {
    if (active_options.count(it->second) == 0)
    {
      // First time we've see this option
      active_options.insert(it->second);
      // creat a new entry in the map
      std::set<std::string> new_set;
      new_set.insert(it->first);
      vehicle_assignment[it->second] = new_set;
    }
    else
    {
      // The map will already have an entry
      vehicle_assignment[it->second].insert(it->first);
    }
  }

  // Step 2. For each active option, find the vehicles
  //         And a total of the input
  std::set<std::string>::iterator its;
  for (its = active_options.begin(); its != active_options.end(); its++)
  {
    std::string option_name = *its; // For clarity

    std::string starting_line = toupper(option_name) + ": ";
    int count = 0;
    std::string list_of_vehicles = "";
    bool top_line_done = false;
    std::set<std::string>::iterator its2;
    for (its2 = vehicle_assignment[option_name].begin();
         its2 != vehicle_assignment[option_name].end(); its2++)
    {
      if (count < 5)
      {
        // add it to this line
        list_of_vehicles += *its2;
        if (count < 4)
          list_of_vehicles += ",";
        count += 1;
      }
      else
      {
        // save the exisint line and start a new one
        if (!top_line_done)
        {
          m_report.push_back(starting_line + list_of_vehicles);
          top_line_done = true;
        }
        else
          m_report.push_back("          " + list_of_vehicles);
        list_of_vehicles = *its2 + ",";
        count = 1;
      }
    }
    // Now check if there are any left over vehicles that did not get posted
    if (list_of_vehicles.size() > 1)
    {
      // remove the comma at the end
      if (count != 5)
        list_of_vehicles = list_of_vehicles.substr(0, list_of_vehicles.size() - 1);
      if (top_line_done)
      {
        m_report.push_back("          " + list_of_vehicles);
      }
      else
      {
        m_report.push_back(starting_line + list_of_vehicles);
      }
    }
    // starting_line += stringSetToString(vehicle_assignment[option_name], ',');

    // m_report.push_back(starting_line);

    // Step 3.  Now get the total input for each option.
    // reuse its2
    for (its2 = m_input_vars.begin(); its2 != m_input_vars.end(); its2++)
    {
      std::string input_var = *its2;
      std::string utility_line = "    " + toupper(*its2) + " = ";
      double total_utility = 0.0;
      double total_vehicles = 0.0;

      std::set<std::string>::iterator its3;
      for (its3 = vehicle_assignment[option_name].begin();
           its3 != vehicle_assignment[option_name].end(); its3++)
      {
        std::string vname = *its3;
        // get the utitlity from this vehicle for this option

        if (m_input_map[tolower(vname)].count(toupper(input_var)) > 0)
        { // sanity check;
          total_utility += m_input_map[tolower(vname)][toupper(input_var)];
          total_vehicles += 1.0;
        }
      }

      double average_utility = 0.0;
      if (total_vehicles > 0.0)
      { // don't divide by zero
        average_utility = total_utility / total_vehicles;
      }
      utility_line += doubleToStringX(average_utility);
      m_report.push_back(utility_line);
    }
  }

  // Post it!
  postViewableText();

  // Do your thing here!
  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool FldEvalOption::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if (param == "input_var")
    {
      m_input_vars.insert(toupper(value));
      handled = true;
    }
    else if (param == "text_x")
    {
      handled = setDoubleOnString(m_text_x, value);
    }
    else if (param == "text_y")
    {
      handled = setDoubleOnString(m_text_y, value);
    }
    else if (param == "text_font_size")
    {
      handled = setIntOnString(m_text_font_size, value);
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void FldEvalOption::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
}

//---------------------------------------------------------
// Procedure: handleMailNodeReport()
//   Example: NAME=alpha,TYPE=KAYAK,UTC_TIME=1267294386.51,
//            X=29.66,Y=-23.49,LAT=43.825089, LON=-70.330030,
//            SPD=2.00, HDG=119.06,YAW=119.05677,DEPTH=0.00,
//            LENGTH=4.0,MODE=ENGAGED

bool FldEvalOption::handleMailNodeReport(const string &node_report_str)
{
  NodeRecord new_record = string2NodeRecord(node_report_str);

  if (!new_record.valid())
  {
    reportRunWarning("ERROR: Unhandled node record");
    return (false);
  }

  // In case there is an outstanding RunWarning indicating the lack
  // of a node report for a given vehicle, retract it here. This is
  // mostly a startup timing issue. Sometimes a sensor request is
  // received before a node report. Only a problem if the node report
  // never comes. Once we get one, it's no longer a problem.
  string vname = new_record.getName();
  retractRunWarning("No NODE_REPORT received for " + vname);

  m_map_node_records[vname] = new_record;

  // Set up interal record keeping for this vehicle
  // Only do it once.
  // Register for incoming option variables if needed
  if (m_vehicles_registered_for_option.count("OPTION_" + tolower(vname)) == 0)
  {

    // Step 1.
    // Options will come in as OPTION_vname
    std::string message_spec = "OPTION_" + tolower(vname);
    Register(message_spec, 0);
    m_vehicles_registered_for_option.insert(message_spec);

    // Step 2.
    // Option inputs will come in as INPUT_VAR_vname
    std::set<std::string>::iterator it;
    for (it = m_input_vars.begin(); it != m_input_vars.end(); it++)
    {
      std::string input_message_spec = toupper(*it) + "_" + tolower(vname);
      Register(input_message_spec, 0);
      m_registered_input_vars_plus_veh.insert(input_message_spec);
    }

    // Step 3.
    std::map<std::string, double> empty_input_map;
    m_input_map[tolower(vname)] = empty_input_map;
  }

  return (true);
}

//------------------------------------------------------------
// Procedure: handleInputVarMsg()
bool FldEvalOption::handleInputVarMsg(std::string key, double value)
{

  // This assumes the map was set up.
  // Step 1.  Get the input var name and the vehicle
  //          Look for INPUT_VAR_VNAME
  std::string vname;
  std::string input_var_name;

  bool valid_msg = false;
  std::map<std::string, std::map<std::string, double> >::iterator it;
  for (it = m_input_map.begin(); it != m_input_map.end(); it++)
  {
    if (strContains(key, it->first))
    {
      // This vehicle name is in this string
      vname = it->first;

      // Now get the name of the input var
      std::set<std::string>::iterator it2;
      for (it2 = m_input_vars.begin(); it2 != m_input_vars.end(); it2++)
      {
        if (strBegins(toupper(key), toupper(*it2)))
        {
          input_var_name = toupper(*it2);
          break;
        }
      }

      valid_msg = true;
      break;
    }
  }

  if (valid_msg == false)
    return (false);

  // If here, the vname and input_var_name are filled out correctly
  m_input_map[tolower(vname)][toupper(input_var_name)] = value;

  return (true);
}

//------------------------------------------------------------
// Procedure: postViewableText()
void FldEvalOption::postViewableText()
{
  // clear existing textbox
  std::string text_box_label = "total_utility";
  XYTextBox new_text_box;
  Notify("VIEW_TEXTBOX", new_text_box.get_spec_inactive());

  // Now build the new textbox
  new_text_box.setX(m_text_x);
  new_text_box.setY(m_text_y);
  new_text_box.setFSize(m_text_font_size);

  for (int i = 0; i < m_report.size(); i++)
  {
    new_text_box.addMsg(m_report[i]);
  }

  Notify("VIEW_TEXTBOX", new_text_box.get_spec());
}

//------------------------------------------------------------
// Procedure: buildReport()

bool FldEvalOption::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  // ACTable actab(4);
  // actab << "Alpha | Bravo | Charlie | Delta";
  // actab.addHeaderLines();
  // actab << "one" << "two" << "three" << "four";
  // m_msgs << actab.getFormattedString();

  for (int i = 0; i < m_report.size(); i++)
  {
    m_msgs << m_report[i] << endl;
  }

  return (true);
}
