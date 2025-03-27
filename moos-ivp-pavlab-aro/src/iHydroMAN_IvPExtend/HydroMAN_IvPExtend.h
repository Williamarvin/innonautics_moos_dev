/************************************************************/
/*    NAME: Supun Randeni                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: HydroMAN_IvPExtend.h                                          */
/*    DATE: July 29th, 2021                             */
/************************************************************/

#ifndef HydroMAN_IvPExtend_HEADER
#define HydroMAN_IvPExtend_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "lib_hydroman_protobuf/hydroman_interface.pb.h"
#include "tcp/tcp_client.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "NodeRecord.h"

using namespace boost;

class HydroMAN_IvPExtend : public AppCastingMOOSApp
{
 public:
   HydroMAN_IvPExtend();
   ~HydroMAN_IvPExtend();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool handleConfigHydroMANGatewayIp(std::string value);
   bool GeodesySetup();
   void connectToiHydroMAN_Gateway();
   void defineIncomingInterfaceMsges();
   void handle_DataFromHydroMAN_interfaceMsges(
    const hydroman::interface::DataFromHydroMAN& msg, const boost::asio::ip::tcp::endpoint& ep);
   void handle_gps_data();
   void handle_no_gps();
   void handle_compass_data(double);
   void handle_actuator_data();
   // bool readConfigTargetVehicles(std::string value);
   void handle_tgt_range_data(std::string sval);
   void handle_tgt_bearing_data(std::string sval);
   void handle_tgt_track_data(std::string sval);

 private: // Configuration variables
  // MOOS Message variable names
  std::string m_var_gps_x = "GPS_X";
  std::string m_var_gps_y = "GPS_Y";
  std::string m_var_gps_lat = "GPS_LAT";
  std::string m_var_gps_lon = "GPS_LON";
  std::string m_var_gps_sat = "GPS_SAT";
  std::string m_var_compass_heading = "COMPASS_HEADING";
  std::string m_var_thrust_left = "PYDIR_THRUST_L";
  std::string m_var_thrust_right = "PYDIR_THRUST_R";

  std::string m_var_tgt_range = "CRS_RANGE_REPORT";
  std::string m_var_tgt_bearing = "CRS_BEARING_REPORT";
  std::string m_var_tgt_track = "UNIT_TRACK_IN";
  bool m_publish_tgt_node_rpt = true;
  bool m_use_p_nav_for_track  = false;

  double gps_expire_time  = 5;
  double m_compass_stdev  = 1;
  double m_pitch_stdev    = 0.5;
  double m_roll_stdev     = 0.5;
  double m_range_stdev    = 3;

 private: // State variables
  // Geodesy related
  CMOOSGeodesy m_geodesy;
  double lat_origin_;
  double lon_origin_;

  // Interface related
  boost::asio::io_service io_;
  std::shared_ptr<hydroman::tcp_client> client_{hydroman::tcp_client::create(io_)};
  unsigned int m_hm_gateway_port = 1101;
  std::string m_hm_gateway_ip_str = "127.0.0.1";

  // Msg content
  struct GPS_STATE {
    double time = 0;
    double x = 0;
    double y = 0;
    double lat = 0;
    double lon = 0;
    int sat = 0;
    bool x_rcvd = false;
    bool y_rcvd = false;
    bool lat_rcvd = false;
    bool lon_rcvd = false;
    bool sat_rcvd = false;
  };
  GPS_STATE m_current_gps;

  struct ACTUATOR_DATA {
    double time = 0;
    double thrust_left = 0;
    double thrust_right = 0;
    bool thrust_left_rcvd = false;
    bool thrust_right_rcvd = false;
  };
  ACTUATOR_DATA m_current_actuator;

  double m_current_compass_hdg = 0;

  // // Target vehicle nav related
  std::string m_ownship_name = "";
  // std::vector<std::string> target_vehicles;
 
};

#endif 
