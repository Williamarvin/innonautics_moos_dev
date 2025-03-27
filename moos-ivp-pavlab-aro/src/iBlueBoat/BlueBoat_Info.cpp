/*****************************************************************/
/*    NAME: Tyler Paine, Mike Defilippo, Mike Benjamin           */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BlueBoat_Info.cpp                                    */
/*    DATE: 25 APRIL 2024                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "BlueBoat_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The iBlueBoat application serves as an intermediary between       ");
  blk("  a MOOS community (backseat) and a vehicle (frontseat).        ");
  blk("                                                                ");
  blk("  iBlueBoat performs two essential tasks:                           ");
  blk("                                                                ");
  blk("    1) CONNECT: [FRONTSEAT] <-(IP:TCP)-> [BACKSEAT]             ");
  blk("                                                                ");
  blk("    2) TRANSLATE: [NMEA MESSAGES] <-(iBlueBoat)-> [MOOS MESSAGES]   ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iBlueBoat file.moos [OPTIONS]                                ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias", "=<ProcessName>                                      ");
  blk("      Launch iBlueBoat with the given process name                  ");
  blk("      rather than iBlueBoat.                                        ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iBlueBoat.                     ");
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
  blu("iBlueBoat Example MOOS Configuration                                ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iM300                                           ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  ip_address     = 192.168.10.1 // frontseat IP address         ");
  blk("  port_number    = 29500        // port number at IP address    ");
  blk("                                                                ");
  blk("                                                                ");
  blk("  max_appcast_events = 8                                        ");
  blk("  max_appcast_run_warnings = 10                                 ");
  blk("                                                                ");
  blk("  warn_bad_nmea_len  = false                                    ");
  blk("  warn_bad_nmea_nend = false                                    ");
  blk("  warn_bad_nmea_rend = false                                    ");
  blk("  warn_bad_nmea_form = false                                    ");
  blk("  warn_bad_nmea_chks = false                                    ");
  blk("  warn_bad_nmea_key  = false                                    ");
  blk("                                                                ");
  blk("                                                                ");
  blk("                                                                ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blu("=============================================================== ");
  blu("iBlueBoat INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk(" The following are examples of possible subscriptions and       ");
  blk(" publications. These need to be implemented per the specific    ");
  blk(" vehicle interface.    ");
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  DESIRED_THRUST        (double) Desired thrust            [%]  ");
  blk("  DESIRED_RUDDER        (double) Desired rudder angle      [deg]");
  blk("                                                                ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  NAV_X                 (double)  X-position on local grid [m]  ");
  blk("  NAV_Y                 (double)  Y-position on local grid [m]  ");
  blk("  NAV_LAT               (double)  Latitude                 [deg]");
  blk("  NAV_LON               (double)  Longitude                [deg]");
  blk("  NAV_SPEED             (double)  Speed                    [m/s]");
  blk("  NAV_HEADING           (double)  Heading from true North  [deg]");
  blk("                                                                ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iBlueBoat", "gpl");
  exit(0);
}
