/****************************************************************/
/*   NAME: Supun Randeni                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: HydroMAN_IvPExtend_Info.cpp                               */
/*   DATE: July 29th, 2021                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "HydroMAN_IvPExtend_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("------------------------------------                            ");
  blk("  The iHydroMAN_IvPExtend serves as an intermediary between     ");
  blk("  the client MOOS community that anticipates to use HydroMAN    ");
  blk("  (e.g. moos-ivp-extend tree), and the HydroMAN MOOS community. ");
  blk("                                                                ");
  blk("  iHydroMAN_IvPExtend ensures the connection between HydroMAN   ");
  blk("  MOOS community and moos-ivp-extend community as follows:      ");
  blk("                                                                ");
  blk("       [HydroMAN] <-> [iHydroMAN_Gateway] <-[IP:TCP]->          ");
  blk("       [iHydroMAN_IvPExtend] <-> [moos-ivp-extend]              ");
  blk("                                                                ");
  blk("                                                                ");
  blk("  iHydroMAN_IvPExtend translates between the protocol buffer    ");
  blk("  messages used by the iHydroMAN_Gateway, and custom MOOS       ");
  blk("  messages, as configured in iHydroMAN_IvPExtend config block   ");
  blk("                                                                ");
  blk("       [Protobuf] <-(iHydroMAN_IvPExtend)-> [MOOS msg]          ");
  blk("                                                                ");
  blk("                                                                ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iHydroMAN_IvPExtend file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iHydroMAN_IvPExtend with the given process name         ");
  blk("      rather than iHydroMAN_IvPExtend.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iHydroMAN_IvPExtend.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iHydroMAN_IvPExtend Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iHydroMAN_IvPExtend                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("  LatOrigin = 42.358436              ");
  blk("  LongOrigin = -71.087448        ");
  blk("                                                                ");
  blk("  hydroman_gateway_port = 1101 ");
  blk("  hydroman_gateway_ip = 127.0.0.1        ");
  blk("                                                                ");
  blk("  // ************************************************************");
  blk("  // OWNSHIP NAVIGATION RELATED ");
  blk("  // *************************************************************");
  blk("                                                                ");
  blk("  gps_expiration_time = 5 ");
  blk("                                                                ");
  blk("  // Prefixes of the ownship sensor data messages, which will be sent to HydroMAN ");
  blk("  prefix_gps_msg = GPS_ ");
  blk("  prefix_imu_msg = IMU_ ");
  blk("  prefix_compass_msg = COMPASS_ ");
  blk("  prefix_actuator_msg = PYDIR_THRUST_ ");
  blk("                                                                ");
  blk("  // Prefixes for nav and target nav output provided by HydroMAN ");
  blk("  // Msg style eg: 'prefix_nav_data' + X/Y/LAT/LONG/HEADING ");
  blk("  prefix_nav_output = HYDROMAN_NAV_           ");
  blk("                                                                ");
  blk("  // *************************************************************");
  blk("  // TARGET VEHICLE NAVIGATION RELATED ");
  blk("  // **************************************************************");
  blk("                                                                ");
  blk("  //target_vehicles=JING,KIRK,IDA,GUS ");
  blk("  //ownship_name=JING ");
  blk("                                                                ");
  blk("  // Prefixes of the target vehicle sensor data messages, which will be sent to HydroMAN ");
  blk("  // Msg style eg: 'range_msg'  ");
  blk("  range_msg = CRS_RANGE_REPORT ");
  blk("  bearing_msg = CRS_BEARING_REPORT  ");
  blk("  track_msg = UNIT_TRACK_IN  ");
  blk("                                                                ");
  blk("  // Msg style eg: 'prefix_target_nav_output' + 'target_name' + X/Y/LAT/LONG/HEADING ");
  blk("  prefix_target_nav_output = HYDROMAN_TARGET_NAV_ ");
  blk("                                                                ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iHydroMAN_IvPExtend INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_MESSAGE = src_node=alpha,dest_node=bravo,var_name=FOO,   ");
  blk("                 string_val=BAR                                 ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  Publications are determined by the node message content.      ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iHydroMAN_IvPExtend", "gpl");
  exit(0);
}

