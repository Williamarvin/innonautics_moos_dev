/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: iSeaTracX150V0.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef SeaTrac_HEADER
#define SeaTrac_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "serial_device.h"
#include "BpSeatrac/seatrac_types.h"
#include "BpSeatrac/seatrac_beacon.hpp"

class iSeaTracX150V0 : public AppCastingMOOSApp
{
public:
  iSeaTracX150V0();
  ~iSeaTracX150V0();

protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

protected:
  void registerVariables();

protected:

  bool connectToSeatrac();

  void sendPing(ESeatracBeaconId id);

  void writeSeaTracPort(pointer context, PSeatracCmdEncodeParams params);
  void disconnectFromSeaTrac();
  void readSeaTracPort();
  void messageDecoded(pointer context, PSeatracCmdDecodeMsgParams params);
  void lineDecoded(pointer context, PSeatracCmdDecodeLineParams params);

  //Sys messages
  void seatracSysAlive(pointer context, PSeatracSysAliveParams params);
  void seatracSysInfo(pointer context, PSeatracSysInfoParams params);
  void seatracStatus(pointer context, PSeatracStatusParams params);

  //Ping Response
  void seatracPingResponse(pointer context, PSeatracPingResponseParams params);
  
private: // Configuration variables

  //Parameters and remappable subs and pubs
  std::string m_nav_x_key;
  std::string m_nav_x_sub;
  double m_nav_x;
  std::string m_nav_y_key;
  std::string m_nav_y_sub;
  double m_nav_y;
  std::string m_nav_hdg_imu_key;
  std::string m_nav_hdg_imu_sub;
  double m_nav_hdg_imu;
  std::string m_nav_hdg_gps_key;
  std::string m_nav_hdg_gps_sub;
  double m_nav_hdg_gps;
  std::string m_port_name_key;
  std::string m_port_name;

  //State variables
  //Utility
  bool m_port_is_opened;
  SerialDevice m_x150_beacon_interface;
  static const uint32 m_interface_buffer_length = 256;
  char m_interface_write_buffer[m_interface_buffer_length];
  CSeatrac *m_x150_object;

  //Data
  double m_ping_send_moos_time;
  double m_ping_resp_moos_time;
  double m_ping_range_time;
  double m_latest_range;
  double m_latest_bearing;
  double m_latest_depth;
  double m_latest_northing;
  double m_latest_easting;
  double m_x150_hdg;
  double m_last_ping;
  double m_next_ping;
  double m_ping_interval;
private: // State variables
  
};

#endif
