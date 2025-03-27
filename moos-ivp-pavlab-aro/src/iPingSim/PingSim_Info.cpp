/****************************************************************/
/*   NAME:                                              */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: PingSim_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "PingSim_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The iPingSim application is used for simulated altimeter      ");
  blk("  measurements taken from a grid                                ");
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
  blu("Usage: iPingSim file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iPingSim with the given process name         ");
  blk("      rather than iPingSim.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iPingSim.        ");
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
  blu("iPingSim Example MOOS Configuration                             ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iPingSim                                        ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("     mode = file   // three options: file, function, or random  ");
  blk("                   // the function option is a hardcoded        ");
  blk("                   // sine wave for now                         ");
  blk("                                                                ");
  blk("      filename = FullGrids/FullGrid$(GRIDFILE)v2.grd            ");
  blk("                                                                ");
  blk("   mirror_grid = (true or false)  (str)                         ");
  blk("                                                                ");
  blk("  // load a grid config for mirroring, follows the same         ");
  blk("  // numbering convention as pRoutePlan                         ");
  blk(" GRID_CONFIG = pts={$(X_1),$(Y_1): $(X_2),$(Y_2): $(X_3),$(Y_3): $(X_4),$(Y_4)} ");
  blk(" GRID_CONFIG = cell_size=$(CELLSIZE)                            ");
  blk(" GRID_CONFIG = cell_vars=depth:0:var:1000                       ");
  blk(" GRID_CONFIG = cell_min=depth:0                                 ");
  blk("                                                                ");
  blk(" #ifdef XMODE SIM                                               ");
  blk("   GRID_CONFIG = cell_max=depth:26                              ");
  blk(" #else                                                          ");
  blk("   GRID_CONFIG = cell_max=depth:26                              ");
  blk(" #endif                                                         ");
  blk(" GRID_CONFIG = cell_min=var:0.00001                             ");
  blk(" GRID_CONFIG = cell_max=var:1000                                ");
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
  blu("iPingSim INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_MESSAGE = src_node=alpha,dest_node=bravo,var_name=FOO,   ");
  blk("                 string_val=BAR                                 ");
  blk("  NAV_X, NAV_Y = Vehicle position (double)                      ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  PING_DISTANCE   = Simulated sensor altitude                   ");
  blk("  PING_CONFIDENCE = Simulated sensor confidence                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iPingSim", "gpl");
  exit(0);
}

