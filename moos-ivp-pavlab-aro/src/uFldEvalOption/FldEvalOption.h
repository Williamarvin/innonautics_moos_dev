/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: FldEvalOption.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef FldEvalOption_HEADER
#define FldEvalOption_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "XYTextBox.h"
#include <string>

class FldEvalOption : public AppCastingMOOSApp
{
public:
  FldEvalOption();
  ~FldEvalOption();

protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();
  bool handleMailNodeReport(const std::string &node_report_str);

protected:
  void registerVariables();
  bool handleInputVarMsg(std::string key, double val);
  void postViewableText();

private: // Configuration variables
  std::set<std::string> m_input_vars;
  double m_text_x;
  double m_text_y;
  int m_text_font_size;

private: // State variables
  std::vector<std::string> m_report;

  std::map<std::string, NodeRecord> m_map_node_records;

  std::set<std::string> m_vehicles_registered_for_option; // Option_Abe
  std::set<std::string> m_registered_input_vars_plus_veh; // Option1_input_Abe
  // Key is vname, value is option
  std::map<std::string, std::string> m_vehicle_opt_status;

  // This nested map holds all the inputs
  // m_input_map[vname][input_var_name] = double value
  std::map<std::string, std::map<std::string, double> > m_input_map;
};

#endif
