/************************************************************/
/*    NAME: Raymond Turrisi                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: HydroLinkArduinoBridge.h                                   */
/*    DATE: June 11th, 2023                                 */
/************************************************************/

#ifndef HydroLink_HEADER
#define HydroLink_HEADER

#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include <unordered_map>
#include <set>
#include <string>
#include <cstdarg> //va_list, va_start, va_end

class ArduinoToRPiMsg
{
public:
  ArduinoToRPiMsg();
  double heading;
  double lng;
  double lat;
  double x;
  double y;
  bool leaking;
  std::string repr();
};

class RPiToArduinoMsg
{
public:
  int r;
  int g;
  int b;

  RPiToArduinoMsg();
  std::string repr();
};

class HydroLinkArduinoBridge : public AppCastingMOOSApp
{
public:
  HydroLinkArduinoBridge();
  ~HydroLinkArduinoBridge();
  std::unordered_map<std::string, std::string> get_params();

protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();
  void registerVariables();
  
protected: //App-specific functions
  int startSerialComms();
  int readSerial();
  int writeSerial();
  int postNodeReport();
  bool GeodesySetup();
  bool dbg_print(const char *format, ...);

private: // Configuration variables
  std::string m_p_baud_rate_key;
  std::string m_p_port_rate_key;
  std::string m_p_heading_offset_key;
  std::string m_p_filtering_gps_key;
  std::string m_p_gps_tc_key;
  std::string m_p_debug_mode_key;

  //Input messages
  std::string m_m_rgb_key;
  
  //Output messages
  std::string m_m_node_report_key;

  std::unordered_map<std::string, std::string> m_params;

  std::set<uint64_t> m_valid_bauds;

private: // State variables
  CMOOSGeodesy m_geodesy;
  int m_baud_rate;
  double m_heading_offset;
  std::string m_serial_port;
  static const int m_o_buff_size = 128;
  static const int m_i_buff_size = 128;
  char m_to_arduino_buffer[m_o_buff_size];
  char m_from_arduino_buffer[m_i_buff_size];
  ArduinoToRPiMsg m_latest_rcvd_msg;
  RPiToArduinoMsg m_latest_sent_msg;
  int m_serial_fd;
  bool m_use_nvg_msg_for_nav_x_nav_y;
  std::string m_nav_prefix;
  std::string m_gps_prefix;
  double m_nav_x;
  double m_nav_y;
  double m_nav_x_prv;
  double m_nav_y_prv;
  double m_nav_long;
  double m_nav_lat;
  double m_nav_heading; 
  std::string m_sys_name;

  bool m_filtering_gps;
  double m_gps_filter_tc;
  double m_last_publish_time;

  bool m_debug;
  FILE *m_debug_stream;
  static const uint16_t m_fname_buff_size = 255;
  char m_fname[m_fname_buff_size];
};
#endif
