/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: HeronSim_Info.cpp                                    */
/*    DATE: Mar 17th 2020                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "HeronSim_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  Simulates the front seat computer of a Clearpath Heron.       ");
  blk("  This app is meant to run in a MOOS community that is also     ");
  blk("  uSimMarine.                                                   ");
  blk("                                                                ");
}

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: uSimHeron file.moos [OPTIONS]                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

void showExampleConfigAndExit()
{
  //   0        1         2         3         4         5         6       
  //   1234567890123456789012345678901234567890123456789012345678901234567
  blk("                                                                   ");
  blu("================================================================== ");
  blu("uSimHeron Example MOOS Configuration                               ");
  blu("================================================================== ");
  blk("                                                                   ");
  blk("ProcessConfig = uSimHeron                                          ");
  blk("{                                                                  ");
  blk("  AppTick   = 10                                                   ");
  blk("  CommsTick = 10                                                   ");
  blk("                                                                   ");
  blk("       PORT = 29500  // Port number on localhost                   ");
  blk("                                                                   "); 
  blk("   mag_declination_deg = -14.2   // (Cambridge)                    ");
  blk("                                                                   ");
  blk("}                                                                  ");
  blk("                                                                   ");
  exit(0);
}

void showInterfaceAndExit()
{
  //   0        1         2         3         4         5         6       
  //   1234567890123456789012345678901234567890123456789012345678901234567
  blk("                                                                   ");
  blu("================================================================== ");
  blu("uSimHeron INTERFACE                                                ");
  blu("================================================================== ");
  blk("                                                                   ");
  showSynopsis();
  blk("                                                                   ");
  blk("SUBSCRIPTIONS:                                                     ");
  blk("------------------------------------                               ");
  blk(" NAV_LAT       Current latitude of simulated vehicle               ");
  blk(" NAV_LONG      Current longitude of simulated vehicle              ");
  blk(" NAV_HEADING   Current heading relative to true N                  ");
  blk(" NAV_SPEED     Current simulated speed in m/s                      ");
  blk("                                                                   ");
  blk("PUBLICATIONS:                                                      ");
  blk("------------------------------------                               ");
  blk(" DESIRED_THRUST_L      Desired thrust to simulate for left motor   ");
  blk(" DESIRED_THRUST_R      Desired thrust to simulate for right motor  ");
  blk("                                                                   ");
  exit(0);
}



