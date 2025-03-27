/****************************************************************/
/*   NAME: Tyler Paine                                          */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: DynamLearning_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "DynamLearning_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pDynamLearning application is used for estimating the     ");
  blk("  forward speed of the ClearPath Heron vehicles.  (It could be  ");
  blk("  repurposed for other vechicles too.)                          ");
  blk("                                                                ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pDynamLearning file.moos [OPTIONS]                       ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pDynamLearning with the given process name         ");
  blk("      rather than pDynamLearning.                               ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pDynamLearning.            ");
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
  blu("pDynamLearning Example MOOS Configuration                       ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pDynamLearning                                  ");
  blk("{                                                               ");
  blu("  AppTick   = 4                                                 ");
  blu("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  // State prefix for learning system dynamics                  ");
  blk("  //  Examples are  NAV or GT or GPS                            ");
  blu("  StateVarPrefix = GT                                           ");
  blk("                                                                ");
  blk("  // Filepath to save the weights                               ");
  blu("  SaveFilePath = /some/path/on/the/vehicle/                     ");
  blk("                                                                ");
  blk("  // Time window in which all measurements must lie to be       ");
  blk("  // considered a single state                                  ");
  blu("  MaxTimeLim     = 0.2                                          ");
  blk("                                                                ");
  blk("  // An estimate in attempted every interval                    ");
  blu("  TimeBetweenEst = 0.4;                                         ");
  blk("                                                                ");
  blk("  //////// RNN Parameters ///////                               ");
  blu("  NumberNeurons = 8                                             ");
  blu("  NumberInputs  = 4                                             ");
  blk("                                                                ");  
  blk("  // These values are used to scale the inputs to the range 0-1.");
  blk("  // It is better to over estimate the max values than to       ");
  blk("  // underestimate.  The units are the same as the inputs.      ");
  blu("  MaxInput1 = 4    // Max Speed                                 ");
  blu("  MaxInput2 = 360  // Max Heading                               ");
  blu("  MaxInput3 = 110  // Max Thrust Left Value (Commanded)         ");
  blu("  MaxInput4 = 110  // Max Thrust Right Value (Commanded)        ");
  blk("                                                                ");
  blk("  // These values are used for the Adams back-propagation       ");
  blk("  // algorithm                                                  ");
  blu("  Alpha   = 0.0005                                              "); 
  blu("  Beta1   = 0.9                                                 ");
  blu("  Beta2   = 0.999                                               ");
  blu("  Epsilon = 0.00000001                                          ");
  blk("                                                                ");
  blk("  ////// AID Parameters ///////                                 ");
  blu("  NumberParamsAID = 4                                           ");
  blk("                                                                ");
  blk("  // Initial estimate of each param                             ");
  blu("  AIDParamInit1 = 0.000001                                      ");
  blu("  AIDParamInit2 = 0.000001                                      ");
  blu("  AIDParamInit3 = -0.05                                         ");
  blu("  AIDParamInit4 = -0.0025                                       ");
  blk("                                                                ");
  blk("  // Adaptation gains                                           ");
  blu("  GainParam1 = 0.0000000000001  // Tiny since thruster commands "); 
  blu("  GainParam2 = 0.0000000000001  //      are 0 to 100            ");
  blu("  GainParam3 = 0.00001                                          ");
  blu("  GainParam4 = 0.000001                                         ");
  blu("          aM = -0.1                                             ");
  blk("                                                                ");
  blk("  ////// RLS Parameters //////                                  ");
  blk("                                                                ");
  blu("  NumberParamsRLS = 8                                           ");
  blu("  OrderRLS        = 0                                           ");
  blu("  ForgetingFactor = 0.995                                       ");
  blu("  RLSParamInit1   = 0.0128                                      ");
  blu("  RLSParamInit2   =-0.00024133                                  ");
  blu("  RLSParamInit3   = 0.0000022296                                ");
  blu("  RLSParamInit4   = 0.0402                                      ");
  blu("  RLSParamInit5   =-0.0011                                      ");
  blu("  RLSParamInit6   = 0.0000079938                                ");
  blu("  RLSParamInit7   = 0.0                                         ");
  blu("  RLSParamInit8   = 0.0                                         ");
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
  blu("pDynamLearning INTERFACE                                        ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("(All are local to the vehicle)                                  ");
  blk("  NAV_SPEED, NAV_HEADING  (Or whatever the NAV prefix is        ");
  blk("                          (e.g. NAV or GPS or GT )              ");
  blk("  PYDIR_THRUST_L, PYDIR_THRUST_R    (For Heron vehicles)        ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  NAV_SPEED_EST_RNN   - Estimate using a Recurrent Neural Net   ");
  blk("  NAV_SPEED_EST_AID   - Estimate using a Adaptive Identifier    ");
  blk("  NAV_SPEED_EST_RLS   - Estimate using a Recursive Least Squares");
  blk("  NAV_SPEED_EST_AVE   - Average of estimates                    ");
  blk("  NAV_SPEED_EST_VAR   - Variance of estimates                   ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pDynamLearning", "gpl");
  exit(0);
}

