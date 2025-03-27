/************************************************************/
/*    NAME: Raymond Turrisi                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: HydroLinkArduinoBridge.cpp                                   */
/*    DATE: June 11th, 2023                                 */
/************************************************************/

/*
  TODO: 
   - Refactor to code to use lib_serialdev
   - Clean up code
   - Once Sam adds current and voltage sensing messages, account for this in message parsing and publish this to db
   - Verify low pass filter
*/

#include <iterator>
#include "MBUtils.h"
#include "LatLonFormatUtils.h"

#include <time.h>
#include "ACTable.h"
#include "HydroLinkArduinoBridge.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NodeRecord.h"

using namespace std;

ArduinoToRPiMsg::ArduinoToRPiMsg()
{
  heading = 0;
  lng = 0;
  lat = 0;
  x = 0;
  y = 0;
  leaking = false;
}

RPiToArduinoMsg::RPiToArduinoMsg()
{
  r = 255;
  g = 155;
  b = 55;
}

std::string ArduinoToRPiMsg::repr()
{
  std::string leak_tok = leaking ? "true" : "false";
  char buff[128];
  memset(buff, 128, '\0');
  snprintf(buff, 128, "heading=%0.2f,long=%f, lat=%f, x=%0.2f, y=%0.2f, leaking=%s", heading, lng, lat, x, y, leak_tok.c_str());
  return std::string(buff);
}

std::string RPiToArduinoMsg::repr()
{
  char buff[32];
  memset(buff, 32, '\0');
  snprintf(buff, 32, "$%d,%d,%d,*", r, g, b);
  return std::string(buff);
}

//---------------------------------------------------------
// Constructor()

HydroLinkArduinoBridge::HydroLinkArduinoBridge()
{

  //Parameter keys
  m_p_baud_rate_key = "baud_rate";
  m_p_port_rate_key = "port";
  m_p_heading_offset_key = "heading_offset";
  m_p_filtering_gps_key = "filtering_gps";
  m_p_gps_tc_key = "gps_filter_time_constant";
  m_p_debug_mode_key = "debug";

  //Default parameters
  m_params[m_p_baud_rate_key] = "9600";
  m_params[m_p_port_rate_key] = "/dev/ttyACM0";
  m_params[m_p_heading_offset_key] = "0";
  m_params[m_p_debug_mode_key] = "false";
  m_params[m_p_filtering_gps_key] = "true";
  m_params[m_p_gps_tc_key] = "1";

  //Message keys
  m_m_rgb_key = "RGB_DISPLAY";

  m_m_node_report_key = "NODE_REPORT_LOCAL";


  m_use_nvg_msg_for_nav_x_nav_y = false;
  m_filtering_gps = true;

  //Acceptable parameter argument lookups

  //Baud rates
  m_valid_bauds.insert(9600);
  m_valid_bauds.insert(14400);
  m_valid_bauds.insert(19200);
  m_valid_bauds.insert(38400);
  m_valid_bauds.insert(57600);
  m_valid_bauds.insert(115200);

  //Misc
  m_nav_prefix = "NAV";
  m_gps_prefix = "GPS";
}

//---------------------------------------------------------
// Destructor

HydroLinkArduinoBridge::~HydroLinkArduinoBridge()
{
  close(m_serial_fd);
}

unordered_map<string, string> HydroLinkArduinoBridge::get_params()
{
  return unordered_map<string, string>(m_params);
}

//---------------------------------------------------------
// Procedure: GeodesySetup()
//   Purpose: Initialize geodesy object with lat/lon origin.
//            Used for LatLon2LocalUTM conversion.

bool HydroLinkArduinoBridge::GeodesySetup()
{
  double LatOrigin = 0.0;
  double LonOrigin = 0.0;

  // Get Latitude Origin from .MOOS Mission File
  bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
  if (!latOK)
  {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return (false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
  if (!lonOK)
  {
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return (false);
  }

  // Initialise CMOOSGeodesy object
  bool geoOK = m_geodesy.Initialise(LatOrigin, LonOrigin);
  if (!geoOK)
  {
    reportConfigWarning("CMOOSGeodesy::Initialise() failed. Invalid origin.");
    return (false);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: dbg_print()
bool HydroLinkArduinoBridge::dbg_print(const char *format, ...)
{
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_fname, "a");
    if (m_debug_stream != nullptr)
    {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      return true;
    }
    else
    {
      reportRunWarning("Debug mode is enabled and file could not be opened\n");
      return false;
    }
  }
  return false;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool HydroLinkArduinoBridge::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
    // TODO: Add error checking

    if (key == m_m_rgb_key)
    {
      string value = msg.GetString();
      // Expecting: RGB_DISPLAY=R,G,B
      // RGB values are between 0 and 255
      string r = biteString(value, ',');
      m_latest_sent_msg.r = stoi(r);
      string g = biteString(value, ',');
      m_latest_sent_msg.g = stoi(g);
      string b = value;
      m_latest_sent_msg.b = stoi(b);
    }
    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool HydroLinkArduinoBridge::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool HydroLinkArduinoBridge::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Get the latest messages from the Arduino
  readSerial();

  // Write the latest messages which are automatically updated OnNewMail
  writeSerial();

  AppCastingMOOSApp::PostReport();

  return (true);
}

//---------------------------------------------------------
// Procedure: postNodeReport()

int HydroLinkArduinoBridge::postNodeReport()
{
  NodeRecord node_report;
  node_report.setName(m_sys_name);
  node_report.setType("ship");
  node_report.setTimeStamp(MOOSTime());
  node_report.setX(m_nav_x);
  node_report.setY(m_nav_y);
  node_report.setLat(m_nav_lat);
  node_report.setLon(m_nav_long);
  node_report.setSpeed(0);
  node_report.setHeading(0);
  node_report.setDepth(0.0);
  Notify(m_m_node_report_key, node_report.getSpec());
  return 0;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool HydroLinkArduinoBridge::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  bool sysnameOk = m_MissionReader.GetValue("Community", m_sys_name);
  if (!sysnameOk)
  {
    reportUnhandledConfigWarning("Not able to get vehicle name");
  }

  // Obtain all the configuration parameters

  set<string> expected_params;
  std::unordered_map<std::string, std::string>::iterator param_it;
  for (param_it = m_params.begin(); param_it != m_params.end(); ++param_it)
  {
    expected_params.insert(param_it->first);
  }

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); ++p)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    bool handled = false;

    // TODO: Change parameter strings with keys
    if (expected_params.count(param) != 0)
    {
      if (param == m_p_baud_rate_key)
      {
        // if (isDouble(value))
        if (true)
        {
          m_baud_rate = stoi(value);
          expected_params.erase(param);
          handled = true;
        }
      }
      else if (param == m_p_port_rate_key)
      {
        if (true)
        {
          m_serial_port = value;
          expected_params.erase(param);
          handled = true;
        }
      }
      else if (param == m_p_heading_offset_key)
      {
        m_heading_offset = stod(value);
        expected_params.erase(param);
        handled = true;
      }
      else if (param == m_p_filtering_gps_key)
      {
        m_filtering_gps = (value == tolower("true")) ? true : false;
        handled = true;
      }
      else if (param == m_p_gps_tc_key)
      {
        m_gps_filter_tc = stod(value);
        handled = true;
      }
      else if (param == "debug")
      {
        m_debug = (value == tolower("true")) ? true : false;
        expected_params.erase(param);
        if (m_debug)
        {
          time_t rawtime;
          struct tm *timeinfo;
          memset(m_fname, m_fname_buff_size, '\0');
          time(&rawtime);
          timeinfo = localtime(&rawtime);
          char fmt[m_fname_buff_size];
          memset(fmt, m_fname_buff_size, '\0');
          strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
          snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                   m_sys_name.c_str(), fmt);
          reportRunWarning("Debug file name: " + string(m_fname));
          dbg_print("Opened file\n");
        }
      }
      handled = true;
    }
    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // TODO: Find out why it always says it uses default parameters for a missing debug file
  if (expected_params.size() != 0)
  {
    string missing = "{";
    set<string>::iterator param_it;
    for (param_it = expected_params.begin(); param_it != expected_params.end(); ++param_it)
    {
      missing += *param_it;
      if ((next(param_it)) != expected_params.end())
        missing += ",";
    }
    missing += "}";
    reportRunWarning("Using default values for parameters: " + missing);
  }
  registerVariables();

  // TODO: Call routine for starting serial communication and exit if
  dbg_print("Opening serial comms\n");
  int serial_state = startSerialComms();
  if (serial_state != 0)
  {
    //TODO?
  }
  // exit(serial_state);

  // Init Geodesy
  GeodesySetup();

  return (true);
}

//---------------------------------------------------------
// Procedure: startSerialComms()

int HydroLinkArduinoBridge::startSerialComms()
{
  m_serial_fd = open(m_serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  dbg_print("Attempted to open: %d\n", m_serial_fd);
  if (m_serial_fd == -1)
  {
    dbg_print("Failed to open file\n");
    reportRunWarning("open_port: Unable to open " + m_serial_port);
    return (-1);
  }

  struct termios options;

  tcgetattr(m_serial_fd, &options); // Get the current options for the port
  if (m_valid_bauds.count(m_baud_rate) > 0)
  {
    cfsetispeed(&options, m_baud_rate);
    cfsetospeed(&options, m_baud_rate);
  }
  else
  {
    reportRunWarning("Invalid Baud Rate: exit -2");
    return (-2);
  }

  options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode

  // Set the new options for the port
  dbg_print("Set new options on the port\n");
  tcsetattr(m_serial_fd, TCSANOW, &options);

  dbg_print("Opening comms was a success\n");
  return 0;
}

//---------------------------------------------------------
// Procedure: readSerial()

int HydroLinkArduinoBridge::readSerial()
{
  // Initialize reading from the buffer
  int n = 0;
  int c_count = 0;

  string msg_queue;

  // Read all the bytes in the buffer until it is empty
  // This extracts bytes from the hardware buffer, building the abstract message queue
  while (true)
  {
    memset(m_from_arduino_buffer, '\0', m_i_buff_size);
    n = read(m_serial_fd, m_from_arduino_buffer, m_i_buff_size);
    dbg_print("Reading from arduino: %d\n", n);
    if (n < 0)
    {
      break;
    }
    else
    {
      msg_queue += string(m_from_arduino_buffer);
      dbg_print("Queue: %s\n", msg_queue.c_str());
      c_count += n;
    }
  }
  tcflush(m_serial_fd, TCIOFLUSH);

  // If nothing was read, and the count is zero, then we had an error reading from the port
  if (n < 0 && c_count == 0)
  {
    // reportRunWarning("Failed to read from Arduino");
    return n;
  }
  else
  {
    // If we did not have an error reading the message, we parse the string
    // This is where we parse the semantic meaning of the messages
    while (true)
    {
      // See if we have a complete message in our buffer
      //$HEADING, LAT, LONG, LEAKING,*
      int start = msg_queue.find("$");
      int end = msg_queue.find("*");
      // If we have a complete message in our buffer, and the front of it is contained, parse this message
      if ((start != string::npos) && (end != string::npos) && (end > start))
      {
        int idx = 0;
        // TODO: This is crappy - could include a checkum, currently just looking at the contents
        dbg_print("Have a complete message - queue: %s\n", msg_queue.c_str());
        string msg = msg_queue.substr(start + 1, end - start - 1);
        dbg_print("Have a complete message - iso: %s\n", msg.c_str());
        if(msg.find("DBG") != string::npos) {
          //We found a debug statement print out, with this, remove it from the message queue and write it to the file for debugging purposes
          dbg_print("Debug message: <%s>\n", msg.c_str());
          biteString(msg_queue, '*');
          continue;
        }
        vector<string> flds = parseString(msg, ',');

        string str_hdg = flds[0];
        string str_lat = flds[1];
        string str_lon = flds[2];
        string str_leak = flds[3];

        if (!isNumber(str_lat))
          return (reportRunWarning("Bad lat: " + msg));
        if (!isNumber(str_lon))
          return (reportRunWarning("Bad Long: " + msg));

        double dbl_lat = stod(str_lat);
        double dbl_lon = stod(str_lon);

        m_nav_lat = dbl_lat;
        m_nav_long = dbl_lon;

        Notify(m_nav_prefix + "_LAT", dbl_lat, "GPS");
        Notify(m_nav_prefix + "_LON", dbl_lon, "GPS");

        double x, y;

        bool ok = m_geodesy.LatLong2LocalGrid(dbl_lat, dbl_lon, y, x);

        if (!ok)
        {
          dbg_print("Converting lat/long to y/x unsuccessful: %f, %f\n", x, y);
        }

        m_nav_x_prv = m_nav_x;
        m_nav_y_prv = m_nav_y;

        if(m_filtering_gps) {
          double alpha;
          double delta_t = MOOSTime() - m_last_publish_time;
          alpha = 1.0 - exp( -1.0 * delta_t / m_gps_filter_tc);
          m_nav_x +=  alpha * (x - m_nav_x_prv);
          m_nav_y +=  alpha * (y - m_nav_y_prv);
        } else {
          m_nav_x = x;
          m_nav_y = y;
        }
        
        m_nav_heading = stod(str_hdg);
        m_latest_rcvd_msg.heading = stod(str_hdg);
        m_latest_rcvd_msg.lng = stod(str_lon);
        m_latest_rcvd_msg.lat = stod(str_lat);
        m_latest_rcvd_msg.x = x;
        m_latest_rcvd_msg.y = y;
        m_latest_rcvd_msg.leaking = str_leak == "1" ? true : false;

        Notify(m_nav_prefix + "_X", x, "GPS");
        Notify(m_nav_prefix + "_Y", y, "GPS");
        Notify(m_nav_prefix + "_SPEED", y, "GPS");
        Notify(m_nav_prefix + "_HEADING", y, "GPS");
        m_last_publish_time = MOOSTime();
        break;
      }
      else if ((start != string::npos) && (end != string::npos) && (end < start))
      {
        // Here we have a message with some artifacts from some message we clipped on the left side
        // Remove those artifacts
        // TODO: Check this
        // We clear up to that point, and then repeat
        string garbage = biteString(msg_queue, '*');
      }
      else
      {
        if (start != string::npos && end == string::npos)
        {
          // A "$" was found, but no "*", remove everything up to "$"
          msg_queue = msg_queue.substr(start);
          break;
        }
        else
        {
          // No valid message found, clear the queue
          msg_queue.clear();
          break;
        }
      }
    }
    // If we are here, it is because we no longer have a complete message in our buffer and all have been read and handled
  }
  // If we are here, it is because we finished reading everything from the hardware buffer, and the software buffer, and it is time to move on
  tcflush(m_serial_fd, TCIOFLUSH);
  return 0;
}

//---------------------------------------------------------
// Procedure: writeSerial()
int HydroLinkArduinoBridge::writeSerial()
{
  string msg = m_latest_sent_msg.repr() + "\n";
  dbg_print("Sending message to arduino: %s\n", msg.c_str());
  int n = write(m_serial_fd, msg.c_str(), msg.size());
  if (n < 0)
  {
    reportRunWarning("Error: Writing to the Arduino unsuccessful- " + to_string(n));
    return -1;
  }
  return 0;
}

//---------------------------------------------------------
// Procedure: registerVariables()
void HydroLinkArduinoBridge::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_m_rgb_key, 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool HydroLinkArduinoBridge::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(1);
  actab << "IO";
  actab.addHeaderLines();
  actab << "RPi -> Arduino: " << m_latest_sent_msg.repr();
  actab << "Arduino -> RPi: " << m_latest_rcvd_msg.repr();
  m_msgs << actab.getFormattedString();
  return (true);
}
