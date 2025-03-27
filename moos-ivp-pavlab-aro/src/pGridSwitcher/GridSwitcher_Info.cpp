/****************************************************************/
/*   NAME: Tyler Paine                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: GridSwitcher_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "GridSwitcher_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pGridSwitcher application is used for switching which grid");
  blk("  is sent to the shoreside for viewing.  This app supports      ");
  blk("  delta grid messages                                           ");
  blk("                                                                ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pGridSwitcher file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pGridSwitcher with the given process name         ");
  blk("      rather than pGridSwitcher.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pGridSwitcher.        ");
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
  blu("pGridSwitcher Example MOOS Configuration                        ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pGridSwitcher                                   ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");

  blk(" // These are the expected grids, the deltas are expected to    ");
  blk(" // have the same name with an added _DELTA at the end.         ");
  blk(" input_grid_vars = VIEW_GRID_CONS_LOCAL, VIEW_GRID_GPR_LOCAL, VIEW_GRID_PSG_LOCAL, VIEW_GRID_OBS_LOCAL ");
  blk("                                                                ");
  blk(" // The output variable name where the switched grid is posted  ");
  blk(" // This name is also appended with _DELTA when sending deltas  ");
  blk("      output_var = VIEW_GRID                                    ");
  blk("                                                                ");
  blk("   switching_var = GRID_SWITCH   // (str) Post to this variable ");
  blk("                                 // to request the grid. I.e.   ");
  blk("                                 // 'gpr', 'cons'               ");
  blk("                                                                ");
  blk("            mode = $(XMODE)      // MODE = SIM will load the    ");
  blk("                                 // ground truth                ");
  blk("                                                                ");
  blk("        // filename for the `ground truth'                      ");
  blk("        filename = FullGrids/FullGrid$(GRIDFILE)v2.grd	       ");
  blk("     mirror_grid = 'true' or 'false'                            ");
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
  blu("pGridSwitcher INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_MESSAGE = src_node=alpha,dest_node=bravo,var_name=FOO,   ");
  blk("                 string_val=BAR                                 ");
  blk("                                                                ");
  blk("   All `input_grid_vars' as specified in the config block       ");
  blk("   All `input_grid_vars' + `_DELTA' as specified                ");
  blk("                                                                ");
  blk("   m_switching_var as specified in the config block             ");
  blk("                                                                ");
  blk("          // If a value of true is posted to these variables,   ");
  blk("          // then this app publishes the final grid values.     ");
  blk("           ROUTE_FOUND = (true or false) (str)                  ");
  blk("     ROUTE_FOUND_OTHER = (true or false) (str)                  ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  `output_var' as specified in the config block, i.e. `VIEW_GRID");
  blk("               The currently selected grid will be sent for     ");
  blk("               display along with the grid deltas on the        ");
  blk("               associated `_DELTA' variable name, i.e.          ");
  blk("               VIEW_GRID_DELTA                                  ");
  blk("                                                                ");
  blk("         GRID_READY  = true when the ground truth grid is loaded");
  blk("  GROUND_TRUTH_DEPTH = a string listing the depth of each grid  ");
  blk("                       cell in the ground truth grid.           ");                      
  blk("                                                                ");
  blk("    These variables are posted after a vehicle finds a route    ");
  blk("                                                                ");
  blk("   FINAL_CONSENSUS_DEPTH = a string listing the depth of each grid");
  blk("                           cell in the final consensus grid     ");
  blk("                                                                ");
  blk("FINAL_CONSENSUS_VARIANCE = a string listing the variance of each");
  blk("                           grid cell in the final consensus grid");
  blk("                                                                ");
  blk("             PATH_VECTOR = a string listing the cell indices in ");
  blk("                           the final path                       ");
  blk("                                                                ");
  blk("               COMPLETED = a string with the vehicle name       ");
  blk("                                                                ");
  blk("      NODE_MESSAGE_LOCAL = Notify the other vehicles we found   ");
  blk("                           a route                              ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pGridSwitcher", "gpl");
  exit(0);
}

