/****************************************************************/
/*   NAME:                                              */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: BathyPath_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "BathyPath_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pBathyPath application is used for calculating a path for ");
  blk("  bathymetric surveying.  Two paths can be generated: a         ");
  blk("  lawnmower based on the number of vehicles, and the current    ");
  blk("  vehicle number, and a Markov Decision Process which uses the  ");
  blk("  grid to generate a path with the best probabilistic reward.   ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pBathyPath file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pBathyPath with the given process name         ");
  blk("      rather than pBathyPath.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pBathyPath.        ");
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
  blu("pBathyPath Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pBathyPath                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("        tree_depth    = 5  // depth of look ahead               ");
  blk("                                                                ");
  blk("   use_UCB_for_reward = 0       // if 0, then MVI will be used. ");
  blk("   prob_scale_factor  = 10000   // Needed because Fast GPR and  ");
  blk("                                // extra consensus iterations   ");
  blk("                                 // skew the variance           ");
  blk(" #ifdef PATHMODE LAWN                                           ");
  blk("                                                                ");
  blk("            lawnmower = 1                                       ");
  blk("                                                                ");
  blk("   number_of_vehicles = $(VNUM)  // number of vehicles deployed ");
  blk("            vehicle_n = $(V_N)   // vehicle's number coefficient");
  blk("                                 // for lawnmower calculations  ");
  blk("         grid_degrees = 0        // rotation of grid            ");
  blk("                                                                ");
  blk(" #elseifdef PATHMODE GREEDY                                     ");
  blk("                                                                ");
  blk("            lawnmower = 0                                       ");
  blk("                                                                ");
  blk(" #endif                                                         ");
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
  blu("pBathyPath INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_MESSAGE = src_node=alpha,dest_node=bravo,var_name=FOO,   ");
  blk("                 string_val=BAR                                 ");
  blk("                                                                ");
  blk("        VIEW_GRID_CONS_LOCAL = Grid for MDP path planner        ");
  blk("  VIEW_GRID_CONS_LOCAL_DELTA = Grid deltas                      ");
  blk("                                                                ");
  blk("   NAV_X, NAV_Y, NAV_HEADING = Vehicle state (double)           ");
  blk("                                                                ");
  blk("                   SURVEYING = Status mode (true or false) (str)");
  blk("                  PATH_FOUND = Status mode (true of false) (str)");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("              PATH_UPDATES = Waypoint updates for both lawnmower");
  blk("                             and MDP paths                      ");
  blk("                                                                ");
  blk("              VIEW_SEGLIST = Path trajectory for visualization  ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pBathyPath", "gpl");
  exit(0);
}

