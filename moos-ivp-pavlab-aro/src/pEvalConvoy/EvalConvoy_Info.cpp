/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: EvalConvoy_Info.cpp                                  */
/*    DATE: July 11th, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "EvalConvoy_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pEvalConvoy application is used for evaluating metrics of ");
  blk("  the Convoy behavior. The Convoy behavior produces raw metrics ");
  blk("  and this app consumes this info and produces further metrics  ");
  blk("  based on time over the life-span of the behavior. This app    ");
  blk("  use the EvalConvoyEngine utlity which is also available post- ");
  blk("  mission with the alogceval utility.                           ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blu("=============================================================== ");
  blu("Usage: pEvalConvoy file.moos [OPTIONS]                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pEvalConvoy with the given process name            ");
  blk("      rather than pEvalConvoy.                                  ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pEvalConvoy.               ");
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
  blu("pEvalConvoy Example MOOS Configuration                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pEvalConvoy                                     ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  recap_var      = CONVOY_RECAP              (default)          ");
  blk("  stat_recap_var = CONVOY_STAT_RECAP         (default)          ");
  blk("  spd_policy_var = CONVOY_SPD_POLICY         (default)          ");
  blk("                                                                ");
  blk("  on_tail_thresh    = 10  meters                                ");
  blk("  alignment_thresh  = 60  degrees                               ");
  blk("  tracking_thresh   = 3   meters                                ");
  blk("  rng_switch_thresh = 2   meters                                ");
  blk("}                                                               ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blu("=============================================================== ");
  blu("pEvalConvoy INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  APPCAST_REG  (pEvalConvoy is an appcasting MOOS App)          "); 
  blk("                                                                ");
  blk("  CONVOY_RECAP = convoy_rng=27.36,vname=eve,rng_delta=2.36,     ");
  blk("                 tail_rng=3.13,tail_ang=2.90,mark_bng=13.27,    ");
  blk("                 trk_err=0.4,almnt=6.2,set_spd=1.5,cnv_avg2=1.1,");
  blk("                 cnv_avg5=1.10,cmode=far,utc=9763348280.8,      ");
  blk("                 mx=30.98,my=-19.38,mid=10,tail_cnt=11,index=14 ");
  blk("                                                                ");
  blk("  CONVOY_STAT_RECAP =  follower=eve,leader=deb,ideal_rng=25.00, ");
  blk("                       compression=0.00,index=0                 ");
  blk("                                                                ");
  blk("  CONVOY_SPD_POLICY = full_stop_rng=2,vname=eve,slower_rng=23,  ");
  blk("                      ideal_rng=25,faster_rng=27,               ");
  blk("                      full_lag_rng=40,lag_spd_delta=2,          ");
  blk("                      max_compress=0.9                          ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  APPCAST  (pEvalConvoy is an appcasting MOOS App)              "); 
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pEvalConvoy", "gpl");
  exit(0);
}


