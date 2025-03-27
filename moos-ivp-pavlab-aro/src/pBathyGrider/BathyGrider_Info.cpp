/****************************************************************/
/*   NAME: Tyler Paine and Nick Gershfeld                       */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: BathyGrider_Info.cpp                                 */
/*   DATE: Spring 2022                                          */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "BathyGrider_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pBathyGrider application is used for building a model     ");
  blk("  of the bathymetry map and sharing that model with neighbors.  ");
  blk("  The model generation is completed using a Fast Gaussian       ");
  blk("  process regression, and the sharing is done via a Modified    ");
  blk("  Kalman consensus.  The technical functions for both are       ");
  blk("  found in the lib_learning library and are linked.              ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pBathyGrider file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pBathyGrider with the given process name         ");
  blk("      rather than pBathyGrider.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pBathyGrider.        ");
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
  blu("pBathyGrider Example MOOS Configuration                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pBathyGrider                                    ");
  blk("{                                                               ");
  blk(" AppTick       = 5                                              ");
  blk(" CommsTick     = 5                                              ");
  blk("                                                                ");
  blk("  THESE SETTINGS WORK FOR THE MIT SAILING PAVILION              ");
  blk("                                                                ");
  blk(" GRID_CONFIG = pts={$(X_1),$(Y_1): $(X_2),$(Y_2): $(X_3),$(Y_3): $(X_4),$(Y_4)} ");
  blk(" GRID_CONFIG = cell_size=$(CELLSIZE)                            ");  
  blk(" GRID_CONFIG = cell_vars=depth:0:var:1000                       ");
  blk(" GRID_CONFIG = cell_min=depth:0                                 ");
  blk(" #ifdef XMODE SIM                                               ");
  blk(" GRID_CONFIG = cell_max=depth:26                                ");
  blk(" conversion_factor = 1                                          ");
  blk(" #else                                                          ");
  blk(" GRID_CONFIG = cell_max=depth:37                                ");
  blk(" conversion_factor = 0.00328      // mm to feet                 ");
  blk(" #endif                                                         ");
  blk(" GRID_CONFIG = cell_min=var:0.00001                             ");
  blk(" GRID_CONFIG = cell_max=var:1000                                ");
  blk("                                                                ");
  blk("  no_data_value = 0                                             ");
  blk("                                                                ");
  blk(" // GPR params                                                  ");
  blk(" time_between_estimates = 0.5     // default                    ");
  blk(" appticks_to_skip = 4                                           ");
  blk("                                                                ");
  blk(" gpr_period = 5                   // default                    ");
  blk(" sensor_variance = 0.1            // default                    "); 
  blk(" kernel_length_scale = 0.005                                    ");
  blk(" variance_threshold = 0.015       // default                    ");
  blk(" omit_list_dist_thresh = 5                                      ");
  blk(" kalman_process_noise = 0.0005                                  ");
  blk("                                                                ");
  blk(" consensus_period   = 40         // how often to attempt        ");
  blk("                                    a consensus                 ");
  blk(" consensus_wait_time = 3         // how long to wait for        ");
  blk("                                    responses before attempting ");
  blk("                                    to fuse estimates           ");
  blk(" consensus_timeout  = 6          // how long to wait until giving");
  blk("                                    up on this round of consensus");
  blk("                                                                ");
  blk(" max_iterations = $(VNUM)        // default, should be at least ");
  blk("                                       the number of vehicles-1 ");
  blk("                                                                ");
  blk(" delta_grid_update_thresh = 0.01 // delta will be send if       ");
  blk("			          // different by more than 0.1 = 10%  ");
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
  blu("pBathyGrider INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_MESSAGE = src_node=alpha,dest_node=bravo,var_name=FOO,   ");
  blk("                 string_val=BAR                                 ");
  blk("  NAV_X, NAV_Y    = Vehicle position (double)                   ");
  blk("  PING_DISTANCE   = Distance measured by the altimeter (double) ");
  blk("  PING_CONFIDENCE = Confidence of the measurement (double)      ");
  blk("  SURVEYING       = Status: true or false (string)              ");
  blk("  EXPLORING       = Status: true or false (string)              ");
  blk("  TRANSITING      = Status: true or false (string)              ");
  blk("                                                                ");
  blk("  NEW_CONSENSUS_REQUESTED = Status from group, true or false (str)");
  blk("  CONSENSUS_DATA          = See format in SimpleKalmanConsensus.h");
  blk("                                                                ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  VIEW_GRID_GPR_LOCAL_DELTA                                     ");
  blk("  VIEW_GRID_GPR_LOCAL          = Veiw grid spec                 ");
  blk("  VIEW_GRID_CONS_LOCAL_DELTA                                    ");
  blk("  VIEW_GRID_CONS_LOCAL         = Veiw grid spec                 ");
  blk("  GRIDCELL_XY                  = x,y pos for each grid          ");
  blk("                                                                ");
  blk("  NODE_MESSAGE_LOCAL      = For NEW_CONSENSUS_REQUESTED,        ");
  blk("                            and CONSENSUS_DATA                  ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pBathyGrider", "gpl");
  exit(0);
}

