/****************************************************************/
/*   NAME: Tyler Paine                                          */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: TrajectTranslate_Info.cpp                            */
/*   DATE: March 14th, 2023                                     */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "TrajectTranslate_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pTrajectTranslate application is used for translating     ");
  blk("  the decisions from pHelmIvP into a smooth trajectory in       ");
  blk("  body-relative coordinates.  The decisions are transformed     ");
  blk("  into local (body-relative) coordinates, and then each output  ");
  blk("  is then filtered using a first-order filter. The time         ");
  blk("  constants used in the filter for each coordinate axis can be  ");
  blk("  configured.                                                   ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pTrajectTranslate file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pTrajectTranslate with the given process name         ");
  blk("      rather than pTrajectTranslate.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pTrajectTranslate.        ");
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
  blu("pTrajectTranslate Example MOOS Configuration                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pTrajectTranslate                               ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blu("   time_window = [0.5]                                          ");
  blk("                                                                ");
  blk("   // Heading rate gain is used to calculate                    ");
  blk("   // desired heading rate with the following formulas:         ");
  blk("   // delta_theta = desired_heading - compass_heading           ");
  blk("   // desired_heading_rate = heading_rate_gain * delta_theta    ");
  blk("   // (and the result is input to the lowpass filter)           ");
  blu("   heading_rate_gain = [0.1]                                    ");
  blk("                                                                ");
  blu("   cutoff_freq_vel_x = [20]  // Hz                              ");
  blu("   cutoff_freq_vel_y = [20]  // Hz                              ");
  blu("   cutoff_freq_vel_z = [20]  // Hz                              ");
  blu("   cutoff_freq_ang_z = [20]  // Hz                              ");
  blk("                                                                ");
  blk("   // Variable for the compass heading                          ");
  blu("   body_heading_var = [COMPASS_HEADING_RAW]                     ");
  blk("                                                                ");
  blk("   // Variable for the body depth                               ");
  blu("   body_depth_var   = [NAV_DEPTH]                               ");
  blk("                                                                ");
  blk("   // Configuration for translation of desired speed into       ");
  blk("   // body-relative x and y velocity:                           ");
  blu("   des_speed_in_surge_only = true or [false]                    ");
  blk("          true -> vel_x = DESIRED_SPEED                         ");
  blk("          false -> DESIRED_SPEED is projected into body-relative");
  blk("                   coordinates                                  ");
  blk("                                                                ");
  blk("   // The depth rate gain is used to calculate                  ");
  blk("   // desired velocity in the Z direction  with the following   ");
  blk("   // formulas:                                                 ");
  blk("   // delta_depth = desired_depth - current_depth               ");
  blk("   // desired_vel_z = depth_rate_gain * delta_depth             ");
  blk("   // (and the result is input to the lowpass filter)           ");
  blu("   depth_rate_gain = [0.1]                                      ");
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
  blu("pTrajectTranslate INTERFACE                                     ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("                                                                ");
  blu("  DESIRED_HEADING = double   (From pHelmIvP)                    ");
  blk("                                                                ");
  blu("  DESIRED_SPEED   = double   (From pHelmIvP)                    ");
  blk("                                                                ");
  blu("  DESIRED_DEPTH   = double   (From pHelmIvP)                    ");
  blk("                                                                ");
  blk("  // body_heading_var as configured.   The default variable is  ");
  blu("  COMPASS_HEADING_RAW = double (Deg north from imu)             ");
  blk("                                                                ");
  blk("  // body_depth_var as configured.   The default variable is    ");
  blu("  NAV_DEPTH       = double (increasing going down)              ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  Smooth trajectory as a twist in body-relative                 ");
  blk("  North East Down (NED) (XYZ) coordinates:                      ");
  blk("  (+X is forward, +Y is starboard, +Z is clockwise)             ");
  blk("  (+Z is down)                                                  ");
  blk("                                                                ");
  blu("  DESIRED_VEL_TWIST_LINEAR_X   (double)                         ");
  blu("  DESIRED_VEL_TWIST_LINEAR_Y   (double)                         ");
  blu("  DESIRED_VEL_TWIST_LINEAR_Z   (double)                         ");
  blu("  DESIRED_VEL_TWIST_ANGULAR_Z  (double)                         ");
  blk("                                                                ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pTrajectTranslate", "gpl");
  exit(0);
}

