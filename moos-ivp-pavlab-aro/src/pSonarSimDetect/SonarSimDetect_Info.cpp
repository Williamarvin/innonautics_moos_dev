/****************************************************************/
/*   NAME: Craig Evans                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: SonarSimDetect_Info.cpp                               */
/*   DATE: May 26, 2020                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "SonarSimDetect_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pSonarSimDetect application is used for               ");
  blk("  Simulating an active sonar detecting a target            ");
  blk("  The app uses a sonar beam width and determines if the sonar    ");
  blk("  would detect a target based of location in the beam.           ");
  blk("  The detection probability is based off a guassian distribution    ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pSonarSimDetect file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pSonarSimDetect with the given process name         ");
  blk("      rather than pSonarSimDetect.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pSonarSimDetect.        ");
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
  blu("pSonarSimDetect Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pSonarSimDetect                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  beam_angle = 30 //Default is 30 degrees                         ");
  blk("  target_name = ADVERSARY //must provide name of target          ");
  blk("  target_depth = 10 //Default is 10 m do not need to provide     ");
  blk("  //if the depth is properly reported in NodeReport                                                             ");
  blk("  sonar_frequency = 1 //Default is 1 Hz                                                            ");
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
  blu("pSonarSimDetect INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT                               ");
  blk("  NAV_X                                ");
  blk("  NAV_Y                                                         ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  DETECTION_REPORT = x;y;time                                   ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pSonarSimDetect", "gpl");
  exit(0);
}

