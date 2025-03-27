/****************************************************************/
/*   NAME: Supun Randeni                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: ChangeMyName_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "ChangeMyName_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The uChangeMyName application is used to echo MOOS variable in"); 
  blk("    a different name                                            ");
  blk("                                                                ");
  blk("  The message content of the 'in' moos variable will be echoed  "); 
  blk("     with a new moos variable name 'out'                        ");
  blk("                                                                ");
  blk("  trigger_type = on_receipt : The message content of 'in' moos variable  ");
  blk("                              will be stored in a buffer and once");
  blk("                              the 'trigger' variable is posted  ");
  blk("                              to the MOOSDB, the latest 'in' message");
  blk("                              will be echoed as 'out' ONLY once ");
  blk("                                                                ");
  blk("  trigger_type = true       : If the 'trigger' variable         ");
  blk("                              is 'true' (e.g. TRIGGER_VAR=true),");
  blk("                              'in' message will be continuously ");
  blk("                              echoed with variable name 'out',  ");
  blk("                              until the 'trigger' variable      ");
  blk("                              becomes 'false'                   ");
  blk("  trigger_type = false      : If the 'trigger' variable is      ");
  blk("                              'false' (e.g. TRIGGER_VAR=false), ");
  blk("                              'in' message will be continuously ");
  blk("                              echoed with variable name 'out',  ");
  blk("                              until the 'trigger' variable      ");
  blk("                              becomes 'true'                    ");
 
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: uChangeMyName file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch uChangeMyName with the given process name         ");
  blk("      rather than uChangeMyName.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of uChangeMyName.        ");
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
  blu("uChangeMyName Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = uChangeMyName                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("   trigger = CRS_RANGE_REPORT_ARCHIE, trigger_type = on_receipt,");
  blk("     in = UNIT_TRACK_IN_BETTY, out = UNIT_TRACK_OUT_ARCHIE      ");
  blk("                                                                ");
  blk("   trigger = DISPLAY_PARTICLES_ARCHIE, trigger_type = false,    ");
  blk("     in = PARTICLES_MEMBERS_IN_ARCHIE, out = VIEW_SEGLIST       ");
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
  blu("uChangeMyName INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  Subscriptions are the 'trigger' and 'in' variables specified  ");
  blk("   in the app config block                                      ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  Publications are the 'out' variables specified in config block");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("uChangeMyName", "gpl");
  exit(0);
}

