/****************************************************************/
/*   NAME: Filip Stromstad                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: DynamicTrafficLight_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "DynamicTrafficLight_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pDynamicTrafficLight application is used for giving       ");
  blk("  vehicles demustering using BHV_DubinsPath a green or a red    ");
  blk("  light, i.e a desired speed. This is the speed policy part of  "); 
  blk("  the demustering. The plan is to make ONE BHV_Demuster that    "); 
  blk("  handles both the speed policy and the dubins trajectory.      ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pDynamicTrafficLight file.moos [OPTIONS]                 ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pDynamicTrafficLight with the given process name   ");
  blk("      rather than pDynamicTrafficLight.                         ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pDynamicTrafficLight.      ");
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
  blu("pDynamicTrafficLight Example MOOS Configuration                 ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pDynamicTrafficLight                            ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  vname = abe                                                   ");
  blk("  max_safety_distance = 3                                       ");
  blk("  min_safety_distance = 3                                       ");
  blk("  max_horizon = 12                                              ");
  blk("  min_horizon = 4                                               ");
  blk("  default_speed = 1.0                                           ");
  blk("  precision = 1                                                 ");
  blk("                                                                ");
  blk("  show_visualization = true                                     ");
  blk("  synchronize = true                                            ");
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
  blu("pDynamicTrafficLight INTERFACE                                  ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  TRAJECTORY_MSG_LOCAL = string/XYSelist (from BHV_DubinsPath)  ");
  blk("  NAV_X = double (from iM300:GNRMC)                             ");
  blk("  NAV_Y = double (from iM300:GNRMC)                             ");
  blk("  NAV_SPEED = double (from iM300:GNRMC)                         ");
  blk("  DEMUSTER_CONFIG = string (from pMarineViwever)                ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  DUBIN_UPDATE = string                                         ");
  blk("  VIEW_POLYGON = string/XYPolygon                               ");
  blk("  VIEW_SEGLIST = string/XYSelist                                ");
  blk("  NODE_MESSAGE_LOCAL = string (see TO/FROM OTHER NODES)         ");
  blk("                                                                ");
  blk("TO/FROM OTHER NODES: (NODE_MESSAGE/uFldMessageHandler)          ");
  blk("------------------------------------                            ");
  blk("    TRAJECTORY_MSG = string/XYSelist                            ");
  blk("    TRAJECTORY_HANDSHAKE = string                               ");
  blk("    SPEED_MSG = double                                          ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pDynamicTrafficLight", "gpl");
  exit(0);
}

