/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: iSeaTracX150V0.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

/*
  Notes
  This is the V0 HydroLink abstraction layer with the iSeaTracX150V0, which is aimed at demonstrating the most basic end-to-end one directional localization capabilities with the system
    - X150 Beacon periodically interrogates the X010 Beacon on the Sea Beaver
    - X150 obtains the depth, northing, and easting values with the extended acoustic nav message
    - Takes the local NAV_X and NAV_Y for HydroLink, and translates the vehicles position, and generates a node report for the Sea Beaver
    - This implementation will remain limited to PING protocol messages for the sake of testing - see 8.1 in dev manual


    Order
    - Connect to USB port
    - connect to iSeaTracX150V0
      - Open serial port
      - Attach methods to the CSeaTrac object
    Loop
      - Ping and interrogate a beacon by ID
        - Define the ID and the type 
          - ID and ST_AMSG_REQX (0x6)
          - ID 
      - Handle Ping Response
        - PSeatracPingResponseParams
          - obj.TSeatracAcoFix
              obj.TSeatracAcoFix.Msg
                obj.TSeatracAcoFix.Msg.DestId - for our use with one x150 - this should always be 15
                obj.TSeatracAcoFix.Msg.SrcId - in the short term, this will always be one
                obj.TSeatracAcoFix.Msg.Type - defines the context for how to handle the response
                    Type -> {ESeatracAcoMsgType}
                obj.TSeatracAcoFix.Info
                  Info -> {TSeatracAcoFixInfo}
                    Relevant fields
                    bool RangeValid;                       				 	 If this bit is set, it indicates the record contains the Range fields below. 
                    uint32 RangeCount;                      				 The number of 16kHz timer intervals that were counted between Request message transmission and Response message reception. 
                    double RangeTime;                       				 The time in seconds derived from the RANGE_COUNT value, and with internal timing offsets and compensation applied. Values are in seconds. 
                    ->float RangeDist;                        				 The resolved line-of-sight distance to the remote beacon, based on the RANGE_TIME and VOS values. Values are in metres. 

                    //USBL Fields
                    bool UsblValid;                         				 If this bit is set, it indicates the record contains the USBL fields below. 
                    uint8 UsblChannels;                     				 The number of USBL receiver channels being used to compute the signal angle. Typically this value is either 3 or 4. 
                    float UsblRssi[ST_USBL_CHANNELS];       				 An array of the received signal strengths for each of the USBL receiver channels, where �x� is the value defined by the CHANNELS field. Values are deci-Bels 
                    ->float UsblAzimuth;                      				 The incoming signal azimuth angle from 0� to 360�. Values are in degrees. 
                    float UsblElevation;                    				 The incoming signal elevation angle from -90� to +90�. Values are in degrees. 

                    //Position Fields
                    ->float PositionEasting;                  				 The Easting distance component of the relative position of the remote beacon to the local beacon computed from the range, incoming signal angle, local beacon depth, attitude and magnetic heading. Value in metres. 
                    ->float PositionNorthing;                 				 The Northing distance component of the relative position of the remote beacon to the local beacon computed from the range, incoming signal angle, local beacon depth, attitude and magnetic heading. Value in metres. 
                    ->float PositionDepth;
              - obj.ESeatracCmdStatus
  Todo's
    [ ] Capture status messages from the X150 Beacon
    [ ] Use the status messages for true heading of the X150 Beacon, and include an assembly heading offset
    [ ] 

   - Params
      - USB Port for iSeaTracX150V0 device
      - Message type (normal, extended)

*/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "iSeaTracX150V0.h"
#include <string.h>
#include <NodeRecord.h>
using namespace std;

//---------------------------------------------------------
// Constructor()

iSeaTracX150V0::iSeaTracX150V0()
{
  // populate state variables
  m_ping_send_moos_time = -1;
  m_ping_resp_moos_time = -1;
  m_ping_range_time = -1;
  m_latest_range = -1;
  m_latest_bearing = -1;
  m_latest_depth = -1;
  m_latest_northing = -1;
  m_latest_easting = -1;
  m_x150_hdg = -1;
  m_last_ping = MOOSTime();
  m_ping_interval = 1.5;
  m_next_ping = m_last_ping+m_ping_interval;

  m_nav_x_key = "x_input";
  m_nav_x_sub = "NAV_X";
  m_nav_y_key = "y_input";
  m_nav_y_sub = "NAV_Y";
  m_nav_hdg_imu_key = "heading_input_imu";
  m_nav_hdg_imu_sub = "NAV_HDG_IMU";
  m_nav_hdg_gps_key = "heading_input_gps";
  m_nav_hdg_gps_sub = "NAV_HDG_GPS";
  m_port_name_key = "serial_device";
  m_port_name = "/dev/ttyS0";

}

//---------------------------------------------------------
// Destructor

iSeaTracX150V0::~iSeaTracX150V0()
{
  m_x150_beacon_interface.closeSerial();
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool iSeaTracX150V0::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if (key == m_nav_x_sub) {
      m_nav_x = msg.GetDouble();
    } else if (key == m_nav_y_sub) {
      m_nav_y = msg.GetDouble();
    } else if (key == m_nav_hdg_gps_sub) {
      m_nav_hdg_gps = msg.GetDouble();
    } else if (key == m_nav_hdg_imu_sub) {
      m_nav_hdg_imu = msg.GetDouble();
    } else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool iSeaTracX150V0::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool iSeaTracX150V0::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  readSeaTracPort();

  if(MOOSTime() > m_next_ping) {
    ESeatracBeaconId dest = (ESeatracBeaconId)1;
    m_x150_object->PingSend(dest, ST_AMSG_REQU);
    m_last_ping = MOOSTime();
    m_next_ping = MOOSTime()+m_ping_interval;
    m_ping_send_moos_time = MOOSTime();
  }

  //Should check to see whether or not we have new data
  NodeRecord node_report;
  node_report.setName("SeaBeaver");
  node_report.setType("auv");
  node_report.setTimeStamp(MOOSTime());
  node_report.setX(m_nav_x+m_latest_easting);
  node_report.setY(m_nav_y+m_latest_northing);

  //TODO: Will need to convert 
  //node_report.setLat(m_nav_lat);
  //node_report.setLon(m_nav_long);

  //TODO: 1) filter the samples and construct a model which estimates the AUV's position, 2) from this model, approximate the speed, 3) will report the AUV's heading in NAVX accom message which can provide X/Y to the AUV while carrying a payload, where the vehicle can report its heading
  node_report.setSpeed(0);
  node_report.setHeading(0);
  node_report.setDepth(m_latest_depth);
  Notify("NODE_REPORT_LOCAL", node_report.getSpec());
  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool iSeaTracX150V0::OnStartUp()
{

  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  // Get parameters
  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if (param == m_nav_x_key)
    {
      m_nav_x_sub = value;
      handled = true;
    }
    else if (param == m_nav_y_key)
    {
      m_nav_y_sub = value;
      handled = true;
    }
    else if (param == m_nav_hdg_gps_key)
    {
      m_nav_hdg_gps_sub = value;
      handled = true;
    }
    else if (param == m_nav_hdg_imu_key)
    {
      m_nav_hdg_imu_sub = value;
      handled = true;
    }
    else if (param == m_port_name_key)
    {
      m_port_name = value;
      handled = true;
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Connect to serial device (which is also a parameter)
  m_x150_beacon_interface.config(m_port_name, B115200, 128);

  if (m_x150_beacon_interface.openSerial() < 0)
  {
    m_port_is_opened = false;
  }

  if(m_port_is_opened) {
    connectToSeatrac();
  } else {
    reportRunWarning("CANNOT CONNECT TO SEATRAC");
    cerr << "CANNOT CONNECT TO SEATRAC\n";
    exit(1);
  }

  registerVariables();

  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void iSeaTracX150V0::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_nav_x_sub, 0);
  Register(m_nav_y_sub, 0);
  Register(m_nav_hdg_gps_sub, 0);
  Register(m_nav_hdg_imu_sub, 0);
}

bool iSeaTracX150V0::connectToSeatrac()
{
  // Open USB port obtained from parameters, report on status
  if (m_port_is_opened)
  {
    // Assign/instantiate the CSeaTrac type to the member variable
    m_x150_object = new CSeatrac;


    // Bind the this classes related methods to the iSeaTracX150V0 object
    m_x150_object->OnCmdEncode.AttachMethod(this, &iSeaTracX150V0::writeSeaTracPort, nullptr);
    m_x150_object->OnCmdDecodeMsg.AttachMethod(this, &iSeaTracX150V0::messageDecoded, nullptr);
    m_x150_object->OnCmdDecodeLine.AttachMethod(this, &iSeaTracX150V0::lineDecoded, nullptr);

    m_x150_object->OnSysAlive.AttachMethod(this, &iSeaTracX150V0::seatracSysAlive, nullptr);
    m_x150_object->OnSysInfo.AttachMethod(this, &iSeaTracX150V0::seatracSysInfo, nullptr);
    m_x150_object->OnStatus.AttachMethod(this, &iSeaTracX150V0::seatracStatus, nullptr);

    //Handlers for receiving a specific signal, after we interrogate a response from a remote beacon
    m_x150_object->OnPingResponse.AttachMethod(this, &iSeaTracX150V0::seatracPingResponse, nullptr);
    //m_x150_object->OnDatReceive.AttachMethod(this, &iSeaTracX150V0::seatracReceiveData, nullptr);
    //m_x150_object->OnNavQueryResponse.AttachMethod(this, &iSeaTracX150V0::, nullptr);
    //m_x150_object->OnStatus.AttachMethod(this, &iSeaTracX150V0::, nullptr);
    //m_x150_object->OnXcvrAnalyseNoise.AttachMethod(this, &iSeaTracX150V0::, nullptr);
  }
  return m_port_is_opened;
}

//Beacon interrogation functions
void iSeaTracX150V0::sendPing(ESeatracBeaconId id)
{
  m_x150_object->PingSend(id, ST_AMSG_REQU);
}

// void iSeaTracX150V0::sendNav(ESeatracBeaconId destId, uint8 queryFlags, puint8 data, uint8 dataLen)
// {
//   m_x150_object->NavStatusSend(id, ST_AMSG_REQU, queryFlags, data, dataLen);
// }

// void iSeaTracX150V0::sendData(ESeatracBeaconId destId, ESeatracAcoMsgType msgType, puint8 data, uint8 dataLen)
// {
//   m_x150_object->DatSend(id, ST_AMSG_REQU, data, dataLen);
// }

// void iSeaTracX150V0::getStatus()
// {
//   m_x150_object->StatusGet();
// }

// void iSeaTracX150V0::analyzeNoise()
// {
//   m_x150_object->XcvrAnalyseNoise();
// }

void iSeaTracX150V0::writeSeaTracPort(pointer context, PSeatracCmdEncodeParams params)
{
  (void)(context);

	// Write the string to the underlying serial port
  memset(m_interface_write_buffer, '\0', m_interface_buffer_length);
  memcpy(m_interface_write_buffer, params->Buffer, params->Length);
  std::string msg = std::string(m_interface_write_buffer);
  
	m_x150_beacon_interface.writeSerial(msg);
}

void iSeaTracX150V0::disconnectFromSeaTrac()
{
  m_x150_beacon_interface.closeSerial();
}
void iSeaTracX150V0::readSeaTracPort()
{
  std::string data;
  std::string err = m_x150_beacon_interface.readSerialAndReturnErr(data);
  if(err.size()) {
    //lib_serialdev returns an error message if there is one, and due to the default non-blocking behavior
    // If nothing is in the buffer, then it will report the resource is temporarily unavailable.
    // With this, it is an insignificant warning and can be skipped for this application
    //TODO: Record exact error message output and use exact equals rather than finding a substring (can't remember the exact output)
    if(err.find("Resource temporarily unavailable") != std::string::npos) {
      reportRunWarning("Error reading X150: " + err);
    }
  } else {
    Notify("X150_RAW_DATA", data);
    m_x150_object->CmdDecode(data);
  }
}
void iSeaTracX150V0::messageDecoded(pointer context, PSeatracCmdDecodeMsgParams params)
{
  //Not used
}
void iSeaTracX150V0::lineDecoded(pointer context, PSeatracCmdDecodeLineParams params)
{
  //Not used
}

// Sys messages
void iSeaTracX150V0::seatracSysAlive(pointer context, PSeatracSysAliveParams params)
{
  //Not implemented
}
void iSeaTracX150V0::seatracSysInfo(pointer context, PSeatracSysInfoParams params)
{
  //TODO
  /*
  Q_UNUSED(context)

	QString hardware = QString::fromLatin1("%1")
			.arg(QString::number(params->Hardware.SerialNumber));

	QString bootloader = QString::fromLatin1("v%1.%2.%3")
			.arg(QString::number(params->BootApp.VersionMajor))
			.arg(QString::number(params->BootApp.VersionMinor))
			.arg(QString::number(params->BootApp.VersionBuild));

	QString application = QString::fromLatin1("v%1.%2.%3")
			.arg(QString::number(params->MainApp.VersionMajor))
			.arg(QString::number(params->MainApp.VersionMinor))
			.arg(QString::number(params->MainApp.VersionBuild));

	double_t hours = params->Seconds / 3600.0F;
	double_t minutes = (hours - (int)hours) * 60.0;
	double_t seconds = (minutes - (int)minutes) * 60.0;

	QString runtime = QString::fromLatin1("%1:%2:%3")
			.arg(QString::number((int)hours))
			.arg(QString::number((int)minutes))
			.arg(QString::number((int)seconds));
  */
}
void iSeaTracX150V0::seatracStatus(pointer context, PSeatracStatusParams params)
{

  //TODO
  /*
  Q_UNUSED(context)

	// Get the flags
	quint8 flags = params->Status.Flags;

	emit onSeatracStatus(params->Status.EnvSupply, params->Status.EnvTemp,
						 params->Status.EnvPressure, params->Status.EnvDepth,
						 params->Status.EnvVos);

	emit onSeatracEulerAngles(params->Status.AttitudeYaw, params->Status.AttitudePitch,
							  params->Status.AttitudeRoll);

	if (IS_BIT_SET(flags, ST_STATUS_FLAGS_BNO055_STATUS_BIT)) {
		emit onSeatracAhrsStatus(params->Status.SysFlag, params->Status.AccFlag,
								 params->Status.MagFlag, params->Status.GyroFlag);
	}

	if (IS_BIT_SET(flags, ST_STATUS_FLAGS_BNO055_QUAT_BIT)) {
		emit onSeatracQuaternions(params->Status.Q0, params->Status.Q1,
								  params->Status.Q2, params->Status.Q3);
	}
	*/
}

// Ping Response
void iSeaTracX150V0::seatracPingResponse(pointer context, PSeatracPingResponseParams params)
{
  (void)(context);

  //TODO:
	/*
	if (params->CmdStatus == ST_CST_OK) {
		this->beep();
	}
	else {
		this->boop();
	}*/
  Notify("X150_RESPONSE", "true");
  m_ping_resp_moos_time = MOOSTime();
  TSeatracAcoFix acofix_ping_response = params->Response;
  TSeatracAcoFixInfo usbl_data = acofix_ping_response.Info;

  if(usbl_data.RangeValid) {
    m_latest_range = usbl_data.RangeDist;
    m_ping_range_time = usbl_data.RangeTime;
  } else {
    reportRunWarning("USBL Range Invalid");
  }
  
  if(usbl_data.PositionValid) {
    if(usbl_data.PositionEnhanced) {
      m_latest_northing = usbl_data.PositionNorthing;
      m_latest_easting = usbl_data.PositionEasting;
      m_latest_depth = usbl_data.PositionDepth;
    } else {
      m_latest_northing = usbl_data.PositionNorthing;
      m_latest_easting = usbl_data.PositionEasting;
      m_latest_depth = usbl_data.PositionDepth;
      reportRunWarning("USBL position derived from unenhanced message");
    }

    //TODO: consider PositionFilterError?
  } else {
    reportRunWarning("USBL Position Invalid");
  }
  
  if(usbl_data.UsblValid) {
    m_latest_bearing = usbl_data.UsblAzimuth;
  } else {
    reportRunWarning("USBL Calculating Values Invalid");
  }
  
  
	//emit pingResponse(params->Response);

}

//------------------------------------------------------------
// Procedure: buildReport()

bool iSeaTracX150V0::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: SeaTracV0                             " << endl;
  m_msgs << "============================================" << endl;

  ACTable x150_state_tab(2);
  x150_state_tab << "X150 State" << "---";
  x150_state_tab.addHeaderLines();
  x150_state_tab << "Connected to Serial: " << (m_port_is_opened ? "Connected" : "Disconnected");
  x150_state_tab << "Compass Heading (NI): " << doubleToString(m_x150_hdg,2);
  //Include the IMU data and other sensors on the X150 (including power supply levels)
  m_msgs << x150_state_tab.getFormattedString();
  m_msgs << endl;
  ACTable x150_comms_tab(2);
  x150_comms_tab << "X150 Comms" << "---";;
  x150_comms_tab.addHeaderLines();
  x150_comms_tab << "Last ping time: " << doubleToString(m_ping_send_moos_time,2);
  x150_comms_tab << "Last ping received: " << doubleToString(m_ping_resp_moos_time,2);
  x150_comms_tab << "Latest range: " << m_latest_range;
  x150_comms_tab << "Latest bearing: " << m_latest_bearing;
  x150_comms_tab << "Latest northing: " << m_latest_northing;
  x150_comms_tab << "Latest easting: " << m_latest_easting;
  m_msgs << x150_comms_tab.getFormattedString();
  m_msgs << endl;
  ACTable x010_tab(2);
  x010_tab << "X010 Info" << "---";;
  x010_tab.addHeaderLines();
  x010_tab << "Has received response: " << (m_ping_resp_moos_time > 0 ? "Yes" : "No");
  x010_tab << "Voltage: " << "Not Implemented yet";
  x010_tab << "Last ping received: " << doubleToString(m_ping_resp_moos_time,2);

  m_msgs << x010_tab.getFormattedString();

  return (true);
}
