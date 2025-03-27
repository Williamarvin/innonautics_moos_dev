/****************************************************************/
/*   NAME: Filip Stromstad                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: DemusterAssign_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "DemusterAssign_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pDemusterAssign application is used for assigning nodes   ");
  blk("  to poses (positions and headings) in a given formation so that");
  blk("  they can safely and efficiently demuster.                     ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pDemusterAssign file.moos [OPTIONS]                      ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pDemusterAssign with the given process name        ");
  blk("      rather than pDemusterAssign.                              ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pDemusterAssign.           ");
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
  blu("pDemusterAssign Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pDemusterAssign                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  margin = 10                                                   ");
  blk("  type = circle                                                 ");
  blk("                                                                ");
  blk("  //Formation specific parameters                               ");
  blk("  circle_radius = 40                                            ");
  blk("  arrow_angle = 90                                              ");
  blk("                                                                ");
  blk("  //Dubin parameters                                            ");
  blk("  dubin_turn_radius = 8                                         ");
  blk("                                                                ");
  blk("  //Assignment parameters                                       ");
  blk("  assignment_algorithm = hungarian_ext                          ");
  blk("  assignment_metric = heading                                   ");
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
  blu("pDemusterAssign INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT = NAME=ben,X=-0.38,Y=-14.21,SPD=0.14,HDG=316.8,   ");
  blk("  ...LAT=42.3,LON=-71.1,TYPE=heron,MODE=MODE@INACTIVE,ALLSTOP=  ");
  blk("  ...NothingToDo,INDEX=1,YAW=2.3,TIME=1716906568.82,LENGTH=1.35}");
  blk("                                                                ");
  blk("  DEMUSTER_ASSIGN = bool (from pMarineViwever)                  ");
  blk("  DEMUSTER_CONFIG = string (from pMarineViwever)                ");
  blk("                                                                ");
  blk("  Temporary:                                                    ");
  blk("  DUBIN_POINTS_LEFT = double (from BHV_DUBINS_PATH)             ");
  blk("  NEW_ANCHOR_POINT = string (from pMarineViwever)               ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  DUBIN_UPDATE (string)                                         ");
  blk("  DEMUSTER (bool)                                               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pDemusterAssign", "gpl");
  exit(0);
}

