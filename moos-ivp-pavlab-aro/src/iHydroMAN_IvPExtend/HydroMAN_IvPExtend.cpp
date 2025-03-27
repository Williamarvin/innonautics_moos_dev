/************************************************************/
/*    NAME: Supun Randeni                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: HydroMAN_IvPExtend.cpp                                        */
/*    DATE: July 29th, 2021                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "HydroMAN_IvPExtend.h"

using namespace std;

//---------------------------------------------------------
// Constructor

HydroMAN_IvPExtend::HydroMAN_IvPExtend()
{
  // This is a fake number since BF doesnt give us sat info. But is required for HydroMAN.
  m_current_gps.sat = 8;
}

//---------------------------------------------------------
// Destructor

HydroMAN_IvPExtend::~HydroMAN_IvPExtend()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool HydroMAN_IvPExtend::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();

    if(key == m_var_gps_x) {
      m_current_gps.x = dval;
      m_current_gps.time = MOOSTime();
      m_current_gps.x_rcvd=true;
    }
    if(key == m_var_gps_y) {
      m_current_gps.y = dval;
      m_current_gps.y_rcvd = true;
    }
    if(key == m_var_gps_lat) {
      m_current_gps.lat = dval;
      m_current_gps.lat_rcvd = true;
    }
    if(key == m_var_gps_lon) {
      m_current_gps.lon = dval;
      m_current_gps.lon_rcvd = true;
    }
    if(key == m_var_gps_sat) {
      m_current_gps.sat = dval;
      m_current_gps.sat_rcvd = true;
    }
    if(key == m_var_compass_heading) {
      handle_compass_data(dval);
    }
    if(key == m_var_thrust_left) {
      m_current_actuator.thrust_left = dval;
      m_current_actuator.thrust_left_rcvd = true;
    }
    if(key == m_var_thrust_right) {
      m_current_actuator.thrust_right = dval;
      m_current_actuator.thrust_right_rcvd = true;
    }
    if(key == m_var_tgt_range) {
      handle_tgt_range_data(sval);
    }
    if(key == m_var_tgt_bearing) {
      handle_tgt_bearing_data(sval);
    }
    if(key == m_var_tgt_track) {
      handle_tgt_track_data(sval);
    }

    // If all msges in the group are received, handle them.
    if (m_current_gps.lat_rcvd && m_current_gps.lon_rcvd){
      handle_gps_data();
    }
    if (m_current_actuator.thrust_left_rcvd && m_current_actuator.thrust_right_rcvd){
      handle_actuator_data();
    }

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool HydroMAN_IvPExtend::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool HydroMAN_IvPExtend::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  if (client_->connected()){
    io_.poll();
  }
  else {
    reportRunWarning("Disconnected from HydroMAN gateway.. ");
    // Try to reconnect
    // TO DO - IF WE RECONNECTED, WE NEED TO MAKE SURE NAV SOLUTION CONTINUES SMOOTHLY
    client_->connect(m_hm_gateway_ip_str, m_hm_gateway_port);
    try 
    {
      io_.poll();
    }
    catch(boost::system::error_code & ec)
    {
      reportRunWarning("Trying to reconnect to HydroMAN gateway.. ");
      std::cout << "Trying to reconnect to HydroMAN gateway. Error = " << ec << "\n";
    }
  }

  // Check if GPS is available
  handle_no_gps();

  // AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool HydroMAN_IvPExtend::OnStartUp()
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
    if(param == "hydroman_gateway_port") {
      handled = setPosUIntOnString(m_hm_gateway_port, value);
    }
    else if(param == "hydroman_gateway_ip") {
      handled = handleConfigHydroMANGatewayIp(value);
    }
    // Prefixes of the ownship sensor data messages
    else if (param == "prefix_gps_msg"){
      m_var_gps_x     = value + "_X";
      m_var_gps_y     = value + "_Y";
      m_var_gps_lat   = value + "_LAT";
      m_var_gps_lon   = value + "_LON";
      m_var_gps_sat   = value + "_SAT";
      handled = true;
    }
    else if (param == "prefix_compass_msg"){
      m_var_compass_heading     = value + "_HEADING";
      handled = true;
    }
    else if (param == "prefix_actuator_msg"){
      m_var_thrust_left     = value + "_L";
      m_var_thrust_right    = value + "_R";
      handled = true;
    }
    else if(param == "gps_expiration_time") {
      handled = setPosDoubleOnString(gps_expire_time, value);
    }
    else if(param == "compass_stdev") {
      handled = setPosDoubleOnString(m_compass_stdev, value);
    }
    else if(param == "range_stdev") {
      handled = setPosDoubleOnString(m_range_stdev, value);
    }
    else if(param == "range_msg") {
      m_var_tgt_range     = value;
      handled = true;
    }
    else if(param == "bearing_msg") {
      m_var_tgt_bearing   = value;
      handled = true;
    }
    else if(param == "target_track_msg") {
      m_var_tgt_track     = value;
      handled = true;
    }
    else if(param == "ownship_name") {
      m_ownship_name     = value;
      handled = true;
    }
    else if(param == "use_particle_nav_for_track") {
      handled = setBooleanOnString(m_use_p_nav_for_track, value);
    }
    else if(param == "publish_target_node_report") {
      handled = setBooleanOnString(m_publish_tgt_node_rpt, value);
    }
    
    // else if(param == "target_vehicles") {
    //   // Extract the names of target vehicles
    //   handled = readConfigTargetVehicles(value);
    // }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Init Geodesy 
  GeodesySetup();

  // Connect to iHydroMAN_Gateway
  connectToiHydroMAN_Gateway();
  
  // Define handles for message types coming through the interface
  defineIncomingInterfaceMsges();
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void HydroMAN_IvPExtend::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_var_gps_x, 0);
  Register(m_var_gps_y, 0);
  Register(m_var_gps_lat, 0);
  Register(m_var_gps_lon, 0);
  Register(m_var_gps_sat, 0);
  Register(m_var_compass_heading, 0);
  Register(m_var_thrust_left, 0);
  Register(m_var_thrust_right, 0);
  Register(m_var_tgt_range, 0);
  Register(m_var_tgt_bearing, 0);
  Register(m_var_tgt_track, 0);
}

// //---------------------------------------------------------
// // 
// bool HydroMAN_IvPExtend::readConfigTargetVehicles(std::string value)
// {
//   vector<string> svector = parseString(value, ',');
//   unsigned int i, vsize = svector.size();

//   if (vsize == 0){
//     reportConfigWarning("No target vehicles configured.");
//     return false;
//   }

//   for(i=0; i<vsize; i++) 
//   {
//     string left  = biteStringX(svector[i], '=');
//         target_vehicles.push_back(left);
//     std::cout<<"Target vehicle "<< i+1 << " = " << left.c_str() << std::endl;
//   }
//   return true;
// }

//---------------------------------------------------------
// Procedure: GeodesySetup
bool HydroMAN_IvPExtend::GeodesySetup()
{
  std::cout << "Initializing Geodesy. "  << std::endl;
  double LatOrigin = 0.0;
  double LonOrigin = 0.0;

  // Get Latitude Origin from .MOOS Mission File
  bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
  if(!latOK) {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return(false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
  if(!lonOK){
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return(false);
  }

  // Initialise CMOOSGeodesy object
  bool geoOK = m_geodesy.Initialise(LatOrigin, LonOrigin);
  if(!geoOK) {
    reportConfigWarning("CMOOSGeodesy::Initialise() failed. Invalid origin.");
    return(false);
  }
  std::cout << "Geodesy Initialized. "  << std::endl;
  return(true);
}

//---------------------------------------------------------
bool HydroMAN_IvPExtend::handleConfigHydroMANGatewayIp(std::string value)
{
  bool all_ok = true;

  // Check if this string contains an IP address
  m_hm_gateway_ip_str = value;
  boost::system::error_code err_code_;
  asio::ip::address hm_gateway_ip = 
    asio::ip::address::from_string(m_hm_gateway_ip_str, err_code_);
  
  if (err_code_.value() != 0) {
    // Provided IP address is invalid. Breaking execution.
    std::cout  << "Failed to parse the hydroman_client_ip address. Error code = "
      << err_code_.value() << ". Message: " << err_code_.message() << " \n";
    reportConfigWarning("Failed to parse the hydroman_client_ip address.");
    all_ok = false;
  }
  else {
    all_ok = true;
  }

  return all_ok;
}

//---------------------------------------------------------
void HydroMAN_IvPExtend::connectToiHydroMAN_Gateway()
{
  bool printed_loop_error = false;
  MOOSTrace("Connecting to the HydroMAN gateway.. \n");
  client_->connect(m_hm_gateway_ip_str, m_hm_gateway_port);
  while (!client_->connected())
  {
    client_->connect(m_hm_gateway_ip_str, m_hm_gateway_port);
    usleep(10000);
    try 
    {
      io_.poll();
    }
    catch(boost::system::error_code & ec)
    {
      if (!printed_loop_error){
        reportRunWarning("Looping until I connect to HydroMAN gateway.. ");
        std::cout << "Looping until I connect to HydroMAN gateway. Error = " << ec << "\n";
        printed_loop_error = true;
      }
    }
  }
  MOOSTrace("Connected to HydroMAN gateway. \n");
}

//---------------------------------------------------------
// Define handles for message types coming through the interface
void HydroMAN_IvPExtend::defineIncomingInterfaceMsges()
{
  // Setting up to all 'handle_DataFromHydroMAN_interfaceMsges' function if a 
  // hydroman::interface::DataFromHydroMAN protobuf message came through the interface
  client_->read_callback<hydroman::interface::DataFromHydroMAN>(
  [this](const hydroman::interface::DataFromHydroMAN& msg, const boost::asio::ip::tcp::endpoint& ep)
  { 
    handle_DataFromHydroMAN_interfaceMsges(msg, ep);
  });       
}

//---------------------------------------------------------
// Define handles for message types coming through the interface
void HydroMAN_IvPExtend::handle_DataFromHydroMAN_interfaceMsges(
  const hydroman::interface::DataFromHydroMAN& msg, const boost::asio::ip::tcp::endpoint& ep)
{
  // *************************************************
  // THIS IS STILL TEMP. WE NEED TO HANDLE THESE MSGES
  // *************************************************
  std::cout << "Data received from HydroMAN: " << ep << std::endl;
  std::cout << "Message: " << msg.ShortDebugString() << std::endl;
  Notify("I_HYDROMAN_RAW_IN", msg.ShortDebugString());

  // Handling HydroMAN nav solution
  if (msg.has_nav()){
    double veh_x{0}, veh_y{0};
    if (!std::isnan(msg.nav().lat()) && !std::isnan(msg.nav().lon()))
    {
      m_geodesy.LatLong2LocalUTM(msg.nav().lat(), msg.nav().lon(), veh_y, veh_x);
      Notify("NAV_X", veh_x);
      Notify("NAV_Y", veh_y);
      Notify("NAV_LAT", msg.nav().lat());
      Notify("NAV_LON", msg.nav().lon());
    }
    if (!std::isnan(msg.nav().depth()))
      Notify("NAV_DEPTH", msg.nav().depth());

    if (!std::isnan(msg.nav().roll()))
      Notify("NAV_ROLL", msg.nav().roll());

    if (!std::isnan(msg.nav().pitch()))
      Notify("NAV_PITCH", msg.nav().pitch());

    // if (!std::isnan(msg.nav().heading()))
    //   Notify("NAV_HEADING", msg.nav().heading());

    if (!std::isnan(msg.nav().vel_u()))
      Notify("NAV_SPEED", msg.nav().vel_u());

    std::string nav_source = "UNKNOWN";
    if (msg.nav().has_pos_source()){
      if (msg.nav().pos_source() == hydroman::interface::NAV::UNKNOWN)
        nav_source = "UNKNOWN";
      if (msg.nav().pos_source() == hydroman::interface::NAV::GPS)
        nav_source = "GPS";
      if (msg.nav().pos_source() == hydroman::interface::NAV::MODEL)
        nav_source = "MODEL";
    }

    // Publish ownship track message 
    if (msg.nav().has_stdev_x() && msg.nav().has_stdev_y() && msg.nav().has_cov_xy() &&
      msg.nav().has_stdev_u() && msg.nav().has_stdev_heading() && !std::isnan(veh_x) && !std::isnan(veh_y))
    {
      std::stringstream str_unitTrack;
      str_unitTrack << "vehicle=" << m_ownship_name
             << ",time=" << std::to_string(msg.time()) 
             << ",x=" << std::to_string(veh_x) 
             << ",y=" << std::to_string(veh_y)
             << ",stdev_x=" << std::to_string(msg.nav().stdev_x()) 
             << ",stdev_y=" << std::to_string(msg.nav().stdev_y()) 
             << ",cov_xy=" << std::to_string(msg.nav().cov_xy()) 
             << ",nav_source=" << nav_source 
             << ",depth=" << std::to_string(msg.nav().depth()) 
             << ",heading=" << std::to_string(msg.nav().heading())
             << ",speed=" << std::to_string(msg.nav().vel_u())
             << ",stdev_speed=" << std::to_string(msg.nav().stdev_u())
             << ",stdev_heading=" << std::to_string(msg.nav().stdev_heading());
      if (!m_use_p_nav_for_track){
        Notify(m_var_tgt_track+"_LOCAL", str_unitTrack.str());
      }
    }
  }

  // Handling target nav solutions
  if (msg.has_target_nav())
  {
    double tgt_x{0}, tgt_y{0};
    if (!std::isnan(msg.target_nav().lat()) && !std::isnan(msg.target_nav().lon()))
    {
      m_geodesy.LatLong2LocalUTM(msg.target_nav().lat(), msg.target_nav().lon(), tgt_y, tgt_x);
      // Publish the tgt nav
      NodeRecord node_report;
      node_report.setName(msg.target_nav().vehicle_name());
      node_report.setTimeStamp(msg.time());
      node_report.setX(tgt_x);
      node_report.setY(tgt_y);
      node_report.setLat(msg.target_nav().lat());
      node_report.setLon(msg.target_nav().lon());
      node_report.setSpeed(msg.target_nav().vel_u());
      node_report.setHeading(msg.target_nav().heading());
      node_report.setDepth(0.0);
      if (m_publish_tgt_node_rpt){
        Notify("NODE_REPORT", node_report.getSpec());
        std::cout << "Target Node Report Published: " << node_report.getSpec() << std::endl;
      }
      else{
        std::cout << "Target Node Report NOT Published: " << node_report.getSpec() << std::endl;
      }
      
      
    }
  }

  // Handling MOOS_PUBLISH messages
  if (msg.has_moos_publish()){
    if(msg.moos_publish().has_msg_name() && msg.moos_publish().has_msg_string())
    {
      Notify(msg.moos_publish().msg_name(), msg.moos_publish().msg_string());
    }
    else if (msg.moos_publish().has_msg_name() && msg.moos_publish().has_msg_double())
    {
      Notify(msg.moos_publish().msg_name(), msg.moos_publish().msg_double());
    }
  }

  // Handling HydroMAN PARTICLE nav solution
  if (msg.has_particle_nav()){
    double veh_x{0}, veh_y{0};
    if (!std::isnan(msg.particle_nav().lat()) && !std::isnan(msg.particle_nav().lon()))
    {
      m_geodesy.LatLong2LocalUTM(msg.particle_nav().lat(), msg.particle_nav().lon(), veh_y, veh_x);
      // Notify("NAV_X", veh_x);
      // Notify("NAV_Y", veh_y);
      // Notify("NAV_LAT", msg.particle_nav().lat());
      // Notify("NAV_LON", msg.particle_nav().lon());
    }
    // if (!std::isnan(msg.particle_nav().depth()))
    //   Notify("NAV_DEPTH", msg.particle_nav().depth());

    // if (!std::isnan(msg.particle_nav().roll()))
    //   Notify("NAV_ROLL", msg.particle_nav().roll());

    // if (!std::isnan(msg.particle_nav().pitch()))
    //   Notify("NAV_PITCH", msg.particle_nav().pitch());

    // if (!std::isnan(msg.particle_nav().heading()))
    //   Notify("NAV_HEADING", msg.particle_nav().heading());

    // if (!std::isnan(msg.particle_nav().vel_u()))
    //   Notify("NAV_SPEED", msg.particle_nav().vel_u());

    std::string nav_source = "UNKNOWN";
    if (msg.particle_nav().has_pos_source()){
      if (msg.particle_nav().pos_source() == hydroman::interface::PARTICLE_NAV::UNKNOWN)
        nav_source = "UNKNOWN";
      if (msg.particle_nav().pos_source() == hydroman::interface::PARTICLE_NAV::GPS)
        nav_source = "GPS";
      if (msg.particle_nav().pos_source() == hydroman::interface::PARTICLE_NAV::MODEL)
        nav_source = "MODEL";
    }

    // Publish ownship track message 
    if (msg.particle_nav().has_stdev_x() && msg.particle_nav().has_stdev_y() && 
      msg.particle_nav().has_cov_xy() && msg.particle_nav().has_stdev_u() && 
      msg.particle_nav().has_stdev_heading() && !std::isnan(veh_x) && !std::isnan(veh_y))
    {
      std::stringstream str_unitTrack;
      str_unitTrack << "vehicle=" << m_ownship_name
             << ",time=" << std::to_string(msg.time()) 
             << ",x=" << std::to_string(veh_x) 
             << ",y=" << std::to_string(veh_y)
             << ",stdev_x=" << std::to_string(msg.particle_nav().stdev_x()) 
             << ",stdev_y=" << std::to_string(msg.particle_nav().stdev_y()) 
             << ",cov_xy=" << std::to_string(msg.particle_nav().cov_xy()) 
             << ",nav_source=" << nav_source 
             << ",depth=" << std::to_string(msg.particle_nav().depth()) 
             << ",heading=" << std::to_string(msg.particle_nav().heading())
             << ",speed=" << std::to_string(msg.particle_nav().vel_u())
             << ",stdev_speed=" << std::to_string(msg.particle_nav().stdev_u())
             << ",stdev_heading=" << std::to_string(msg.particle_nav().stdev_heading());
      if (m_use_p_nav_for_track){
        Notify(m_var_tgt_track+"_LOCAL", str_unitTrack.str());
      }
    }
  }
}

//---------------------------------------------------------
// Handle GPS sensor data
void HydroMAN_IvPExtend::handle_gps_data()
{
  // std::cout << "Handling GPS data: " << std::endl;
  m_current_gps.x_rcvd = false;
  m_current_gps.y_rcvd = false;
  m_current_gps.lat_rcvd = false;
  m_current_gps.lon_rcvd = false;
  m_current_gps.sat_rcvd = false;

  // GPS data
  hydroman::interface::DataToHydroMAN gps_data_;
  gps_data_.set_time(m_current_gps.time);

  // double veh_lat{0}, veh_lon{0};
  // try
  // {
  //   m_geodesy.UTM2LatLong(m_current_gps.x, m_current_gps.y, veh_lat, veh_lon);
  // }
  // catch (std::exception& e)
  // {
  //   // reportRunWarning("Error in UTM2LatLong conversion.. ");
  //   // std::cout << "Error in UTM2LatLong conversion. Error = " << e.what() << "\n";
  // }
  

  gps_data_.mutable_gps_data()->set_lat(m_current_gps.lat);
  gps_data_.mutable_gps_data()->set_lon(m_current_gps.lon);
  // This is a fake number since BF doesnt give us sat info. But is required for HydroMAN.
  gps_data_.mutable_gps_data()->set_sat(m_current_gps.sat); 
  gps_data_.mutable_gps_data()->set_fix(true);

  if (client_->connected() && gps_data_.has_gps_data())
  {
    // std::cout << "Sending data to HydroMAN: GPS: " << gps_data_.ShortDebugString() << std::endl;
    Notify("I_HYDROMAN_RAW_OUT", gps_data_.ShortDebugString());
    client_->write(gps_data_);
  }
}

//---------------------------------------------------------
// Check if GPS is available, if not, inform hydroman
void HydroMAN_IvPExtend::handle_no_gps()
{
  // GPS data
  hydroman::interface::DataToHydroMAN gps_data_;
  gps_data_.set_time(m_current_gps.time);

  if (MOOSTime()-m_current_gps.time > gps_expire_time)
  {
    gps_data_.mutable_gps_data()->set_fix(false);
  }
  if (client_->connected() && gps_data_.has_gps_data())
  {
    // std::cout << "Sending data to HydroMAN: GPS: " << gps_data_.ShortDebugString() << std::endl;
    Notify("I_HYDROMAN_RAW_OUT", gps_data_.ShortDebugString());
    client_->write(gps_data_);
  }
}

//---------------------------------------------------------
// Handle compass data, and send to HydroMAN
void HydroMAN_IvPExtend::handle_compass_data(double hdg_)
{
  hydroman::interface::DataToHydroMAN imu_data_;
  imu_data_.set_time(MOOSTime());
  imu_data_.mutable_imu_data()->set_roll(0.0);    // Assuming ASV
  imu_data_.mutable_imu_data()->set_pitch(0.0);   // Assuming ASV
  imu_data_.mutable_imu_data()->set_heading(hdg_);
  imu_data_.mutable_imu_data()->set_stdev_heading(m_compass_stdev);
  imu_data_.mutable_imu_data()->set_stdev_pitch(m_pitch_stdev);
  imu_data_.mutable_imu_data()->set_stdev_roll(m_roll_stdev);

  if (client_->connected() && imu_data_.has_imu_data())
  {
    std::cout << "Sending data to HydroMAN: IMU: " << imu_data_.ShortDebugString() << std::endl;
    Notify("I_HYDROMAN_RAW_OUT", imu_data_.ShortDebugString());
    client_->write(imu_data_);
  }
}

//---------------------------------------------------------
// Handle actuator data
void HydroMAN_IvPExtend::handle_actuator_data()
{
  m_current_actuator.thrust_left_rcvd = false;
  m_current_actuator.thrust_right_rcvd = false;

  hydroman::interface::DataToHydroMAN actuator_data_;
  actuator_data_.set_time(MOOSTime());

  actuator_data_.mutable_actuator_data()->set_left_rpm(m_current_actuator.thrust_left);
  actuator_data_.mutable_actuator_data()->set_right_rpm(m_current_actuator.thrust_right);

  if (client_->connected() && actuator_data_.has_actuator_data())
  {
    Notify("I_HYDROMAN_RAW_OUT", actuator_data_.ShortDebugString());
    client_->write(actuator_data_);
  }
}

//---------------------------------------------------------
// Handle target range data
void HydroMAN_IvPExtend::handle_tgt_range_data(std::string sval)
{
  hydroman::interface::DataToHydroMAN range_report_;
  std::string vehicle;
  double val;
  if(MOOSValFromString(val,sval,"time"))
  {
    range_report_.set_time(val);          
  }
  if(MOOSValFromString(vehicle,sval,"target"))
  {
    string vname = biteStringX(vehicle, '_');
    range_report_.mutable_target_range_data()->set_vehicle_name(vname);
  }
  if(MOOSValFromString(val,sval,"range"))
  {
    range_report_.mutable_target_range_data()->set_range(val);          
  }
  range_report_.mutable_target_range_data()->set_stdev_range(m_range_stdev);

  if (client_->connected() && range_report_.has_target_range_data())
  {
    Notify("I_HYDROMAN_RAW_OUT", range_report_.ShortDebugString());
    client_->write(range_report_);
  } 
}

//---------------------------------------------------------
// Handle target bearing data
void HydroMAN_IvPExtend::handle_tgt_bearing_data(std::string sval)
{
  hydroman::interface::DataToHydroMAN bearing_report_;
  std::string vehicle;
  double val;
  if(MOOSValFromString(val,sval,"time"))
  {
    bearing_report_.set_time(val);          
  }
  if(MOOSValFromString(vehicle,sval,"target"))
  {
    bearing_report_.mutable_target_bearing_data()->set_vehicle_name(vehicle);
  }
  if(MOOSValFromString(val,sval,"bearing"))
  {
    bearing_report_.mutable_target_bearing_data()->set_bearing(val);          
  }
  // TO DO - adding bearing stdev
  // range_report_.mutable_target_bearing_data()->set_stdev_bearing(5); 

  if (client_->connected() && bearing_report_.has_target_bearing_data())
  {
    Notify("I_HYDROMAN_RAW_OUT", bearing_report_.ShortDebugString());
    client_->write(bearing_report_);
  }
}

//---------------------------------------------------------
// Handle target track data
void HydroMAN_IvPExtend::handle_tgt_track_data(std::string sval)
{
  hydroman::interface::DataToHydroMAN track_report_;
  double val;
  double utm_x_;
  double utm_y_;
  std::string vehicle, nav_src;
  if(MOOSValFromString(val,sval,"time"))
  {
    track_report_.set_time(val);          
  }
  if(MOOSValFromString(vehicle,sval,"vehicle"))
  {
    track_report_.mutable_target_track_data()->set_vehicle_name(vehicle.c_str());
  }           
  if(MOOSValFromString(val,sval,"x"))
  {
    utm_x_ = val;          
  }
  if(MOOSValFromString(val,sval,"y"))
  {
    utm_y_ = val;     
    double veh_lat{0}, veh_lon{0};
    m_geodesy.UTM2LatLong(utm_x_, utm_y_, veh_lat, veh_lon);
    track_report_.mutable_target_track_data()->set_lat(veh_lat);
    track_report_.mutable_target_track_data()->set_lon(veh_lon);      
  }
  if(MOOSValFromString(val,sval,"depth"))
  {
    track_report_.mutable_target_track_data()->set_depth(val);          
  }
  if(MOOSValFromString(val,sval,"stdev_x"))
  {
    track_report_.mutable_target_track_data()->set_stdev_x(val);           
  }
  if(MOOSValFromString(val,sval,"stdev_y"))
  {
    track_report_.mutable_target_track_data()->set_stdev_y(val);          
  }
  if(MOOSValFromString(val,sval,"cov_xy"))
  {
    track_report_.mutable_target_track_data()->set_cov_xy(val);           
  }
  if(MOOSValFromString(nav_src,sval,"nav_source"))
  {
    if (nav_src=="UNKNOWN")
      track_report_.mutable_target_track_data()->set_pos_source(hydroman::interface::TARGET_TRACK::UNKNOWN);         

    if (nav_src=="GPS")
      track_report_.mutable_target_track_data()->set_pos_source(hydroman::interface::TARGET_TRACK::GPS);  

    if (nav_src=="MODEL")
      track_report_.mutable_target_track_data()->set_pos_source(hydroman::interface::TARGET_TRACK::MODEL);  
  }
  if(MOOSValFromString(val,sval,"heading"))
  {
    track_report_.mutable_target_track_data()->set_heading(val);           
  }
  if(MOOSValFromString(val,sval,"speed"))
  {
    track_report_.mutable_target_track_data()->set_vel_u(val);           
  }
  if(MOOSValFromString(val,sval,"stdev_speed"))
  {
    track_report_.mutable_target_track_data()->set_stdev_u(val);           
  }
  if(MOOSValFromString(val,sval,"stdev_heading"))
  {
    track_report_.mutable_target_track_data()->set_stdev_heading(val);           
  }

  // Filtering out ownship track solutions
  if (client_->connected() && track_report_.target_track_data().vehicle_name()!=m_ownship_name
    && track_report_.has_target_track_data())
  {
    Notify("I_HYDROMAN_RAW_OUT", track_report_.ShortDebugString());
    client_->write(track_report_);
  }
}

//------------------------------------------------------------
// Procedure: buildReport()

bool HydroMAN_IvPExtend::buildReport() 
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




