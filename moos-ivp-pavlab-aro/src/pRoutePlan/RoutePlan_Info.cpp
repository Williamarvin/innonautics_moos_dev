/****************************************************************/
/*   NAME:                                              */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: RoutePlan_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "RoutePlan_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pRoutePlan application is used for both path planning and ");
  blk("  coordinating paths among all vehicles within communication    ");
  blk("  range.  Path planning is accomplished by A* to find the best  ");
  blk("  path that may still be viable.  Coordination is done by       ");
  blk("  proposing paths with a cost estimate and deciding weather to  ");
  blk("  pursue the propopsed path depending on the other path         ");
  blk("  proposals received                                            ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pRoutePlan file.moos [OPTIONS]                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pRoutePlan with the given process name             ");
  blk("      rather than pRoutePlan.                                   ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pRoutePlan.                ");
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
  blu("pRoutePlan Example MOOS Configuration                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pRoutePlan                                      ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  proposal_var_name   = PROPOSED_PATH   // default              ");
  blk("                                                                ");
  blk("  consensus_wait_time = 5.0 // time to wait after consensus itr ");
  blk("                            // is posted, ensures route plan    ");
  blk("                            // runs after the last consensus    ");
  blk("                                                                ");
  blk("                                                                ");
  blk("  proposal_wait_time  = 2.0  // time to wait to recieve proposals");
  blk("                                                                ");
  blk("  max_proposal_iterations =  10  // defaults                    ");
  blk("                                                                ");
  blk("          depth_threshold = 20.0   // value threshold           ");
  blk("   variance_pct_threshold = 0.33   // dynamic variance threshold");
  blk("                                     cells with depth less than ");
  blk("                                     the depth_threshold, and   ");
  blk("                                     have variance in the bottom");
  blk("                                     0.xx of the range will be  ");
  blk("                                     considered obstacles       ");
  blk("                                                                ");
  blk("        max_maintain_path = 5    // maximum number of times to  ");
  blk("                                 maintain current path before   ");
  blk("                                 switching                      ");
  blk("   explore_possible_paths = 1  // 1 to explore, 0 to not        ");
  blk("                               // if true, will send paths      ");
  blk("                               // to waypoint behavior          ");
  blk("                                                                ");
  blk("       number_of_vehicles = $(VNUM)  // number of vehicles      ");
  blk("                                       deployed                 ");
  blk("                                                                ");
  blk("                vehicle_n = $(V_N) // vehicle's N coefficient for");
  blk("                                      lawnmower calculations     ");
  blk("                                                                ");
  blk("   //  Set the start and goal cells here if desired             ");
  blk("   //  Grid layout (see launch file for details                 ");
  blk("   //   (1) ----------- (4)                                     ");
  blk("   //    |               |                                      ");
  blk("   //    |               |                                      ");
  blk("   //    |               |                                      ");
  blk("   //   (2) ----------- (3)                                     ");
  blk("   //                                                           ");
  blk("   //   Does not have to be horizontal!!                        ");
  blk("                                                                ");
  blk("   // use all the cells along the line 1 to 4                   ");
  blk("   // as possible start cells                                   ");
  blk("      use_all_top_cells_as_start = true                         ");
  blk("                                                                ");
  blk("   // use all the cells along the line 2 to 3                   ");
  blk("   // as possible goal cells                                    ");
  blk("      use_all_bottom_cells_as_goal = true                       ");
  blk("                                                                ");
  blk("   // In addition, you can specify specific cell numbers        ");
  blk("   // for start and goal cells.  These are additive,            ");
  blk("   // so for instance setting use_all_top_cells_as_start = true ");
  blk("   // and specifying cells here will cause the path planner to  ");
  blk("   // consider both groups of cells to be possible starts.      ");
  blk("   // start_cells = 1,2,3,4,                                    ");
  blk("   // goal_cells = 5,6,7,8                                      ");
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
  blu("pRoutePlan INTERFACE                                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_X, NAV_Y  = Vehicle position (double)                     ");
  blk("  PROPOSED_PATH = Format example:                               ");
  blk("                  vname=abe;1,2,3,4,5,6;123.4  (example)        ");
  blk("                  where the path is 1,2,3,4,5,6                 ");
  blk("                  the cost is 123.4  (name is parameter)        ");
  blk("                                                                ");
  blk("  VIEW_GRID_CONS_LOCAL        = Consensus grid                  ");
  blk("  VIEW_GRID_CONS_LOCAL_DELTA  = Consensus grid delta            ");
  blk("                                                                ");
  blk("     SURVEYING   = Vehicle status, (true or false) (string)     ");
  blk("     EXPLORING   = Vehicle status, (true or false) (string)     ");
  blk("                                                                ");
  blk("     CYCLE_INDEX = Waypoint behavior cycle index                ");
  blk("                                                                ");
  blk("     ADD_START     = Add grid cell that contains point          ");
  blk("                     'X.XXX,Y.YYY' to the start list (string)   ");
  blk("     ADD_END       = Add grid cell that contains point          ");
  blk("                     'X.XXX,Y.YYY' to the end list (string)     ");
  blk("     CLEAR_START   = Add grid cell that contains point          ");
  blk("                     'X.XXX,Y.YYY' to the start list (string)   ");
  blk("     CLEAR_END     = Add grid cell that contains point          ");
  blk("                     'X.XXX,Y.YYY' to the start list (string)   ");
  blk("                                                                ");  
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("     PROPOSED_PATH  =  sent to other vehicles via NODE_MESSAGE  ");
  blk("                       (see definition above)                   ");
  blk("        VIEW_CIRCLE  = XY circle spec to mark route             ");
  blk("        PATH_UPDATES = Updated path for waypoints               ");
  blk("                                                                ");
  blk("          ROUTE_FOUND = '1' posted when (as a string value)     ");
  blk("          FINAL_ROUTE = an ordered list of grid cells (string)  ");
  blk("                                                                ");
  blk("               RETURN = 'true' when the route is found (string) ");
  blk("        TIME_TO_ROUTE = time elasped (string)                   ");
  blk("           PATH_FOUND = 'true' when route is found (string)     ");
  blk("    FINAL_OBSTACLES  = a list of grid cell numbers that were    ");
  blk("                       considered obstacles                     ");
  blk("                                                                ");
  blk("    VIEW_GRID_OBS_LOCAL = Grid spec of all obstacles in grid    ");
  blk("                                                                ");
  blk("                                                                ");
  blk("                                                                ");
  blk("                                                                ");
  blk("                                                                ");
   
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pRoutePlan", "gpl");
  exit(0);
}

