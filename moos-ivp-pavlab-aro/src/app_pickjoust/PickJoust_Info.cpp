/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: PickJoust_Info.cpp                                   */
/*    DATE: Oct 28th, 2019                                       */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/

#include <cstdlib> 
#include <iostream>
#include "ColorParse.h"
#include "ReleaseInfo.h"
#include "PickJoust_Info.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pickjoust is a utility for generating starting points for     ");
  blk("  simulated vehicles. Vehicle positions are on a given circle.  ");
  blk("  Also created for each vehicle are (a) a destination waypoint, ");
  blk("  (b) a starting speed (c) a starting heading. The starting     ");
  blk("  heading is by default pointed toward the destination point.   ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  cout << "=====================================================" << endl;
  cout << "Usage: pickjoust [OPTIONS]                           " << endl;
  cout << "=====================================================" << endl;
  cout << "                                                     " << endl;
  showSynopsis();
  cout << "                                                     " << endl;
  cout << "Options:                                             " << endl;
  cout << "  --help, -h                                         " << endl;
  cout << "     Display this help message.                      " << endl;
  cout << "  --version,-v                                       " << endl;
  cout << "     Display the release version of pickpos.         " << endl;
  cout << "  --verbose                                          " << endl;
  cout << "     Produce more verbose output                     " << endl;
  cout << "  --amt=<num>                                        " << endl;
  cout << "     Specify number of points to make (default=10)   " << endl;
  cout << "  --ang_min_diff                                     " << endl;
  cout << "     Minimum angular difference between starting pts " << endl;
  cout << "     on the circle, in degrees. Default is -1 (off)  " << endl;
  cout << "  --ang_max_diff                                     " << endl;
  cout << "     Minimum angular difference between starting pts " << endl;
  cout << "     on the circle, in degrees. Default is -1 (off)  " << endl;
  cout << "  --max_tries=<num>                                  " << endl;
  cout << "     The number of times the random number generator " << endl;
  cout << "     will retry to make a new random point if the one" << endl;
  cout << "     chosen violates the min_ or max_ang_diff        " << endl;
  cout << "     The default value is 1000.                      " << endl;
  cout << "  --psnap=<num>                                      " << endl;
  cout << "     Snap value for rounding x,y coords. The default " << endl;
  cout << "     is 1, which rounds to nearest integer. A value  " << endl;
  cout << "     of 0.1 rounds to the nearest tenth and so on.   " << endl;
  cout << "  --hsnap=<num>                                      " << endl;
  cout << "     Snap value for rounding heading val. The default" << endl;
  cout << "     is 1, which rounds to nearest integer. A value  " << endl;
  cout << "     of 0.1 rounds to the nearest tenth and so on.   " << endl;
  cout << "  --ssnap=<num>                                      " << endl;
  cout << "     Snap value for rounding speed val. Default is   " << endl;
  cout << "     0.1, which rounds to nearest 1/10th meters/sec. " << endl;
  cout << "  --hdg=<config>                                     " << endl;
  cout << "     Specify how initial heading vals are chosen.    " << endl;
  cout << "     Examples:                                       " << endl;
  cout << "        --hdg=rand                                   " << endl;
  cout << "        --hdg=rand,45,90                             " << endl;
  cout << "        --hdg=cent                                   " << endl;
  cout << "        --hdg=cent,-10,10                            " << endl;
  cout << "        --hdg=dest                                   " << endl;
  cout << "        --hdg=dest,-10,10                            " << endl;
  cout << "                                                     " << endl;
  cout << "  --pfile=<filename>                                 " << endl;
  cout << "    Set the output file for chosen starting points.  " << endl;
  cout << "  --dfile=<filename>                                 " << endl;
  cout << "    Set the output file for chosen destination pts.  " << endl;
  cout << "  --sfile=<filename>                                 " << endl;
  cout << "    Set the output file for chosen speeds            " << endl;
  cout << "  --hfile=<filename>                                 " << endl;
  cout << "    Set the output file for chosen headings          " << endl;
  cout << "  --cfile=<filename>                                 " << endl;
  cout << "    Set the output file for chosen circle params     " << endl;
  cout << "  --df, -df, --default_files                         " << endl;
  cout << "    Use the default filenames for all params         " << endl;
  cout << "    seeds_circ.txt, seeds_dest.txt, seeds_hdg.txt,   " << endl;
  cout << "    seeds_pos.txt, seeds_spd.txt                     " << endl;
  cout << "                                                     " << endl;
  cout << "  --spd=<config>                                     " << endl;
  cout << "     Specify how initial speed values are chosen.    " << endl;
  cout << "     Examples:                                       " << endl;
  cout << "        --spd=1,5                                    " << endl;
  cout << "                                                     " << endl;
  cout << "   --hdrs                                            " << endl;
  cout << "     Include header info at top of output files to   " << endl;
  cout << "     indicate source of file, and cmdline args used. " << endl;
  cout << "                                                     " << endl;
  cout << "                                                     " << endl;
  cout << "Examples:                                            " << endl;
  cout << "  pickjoust --amt=5 --circle=x=1,y=2,rad=50          " << endl;
  cout << "  pickjoust --amt=5 --circle=x=1,y=2,rad=50          " << endl;
  cout << "  pickjoust --amt=5 --circle=x=1,y=2,rad=50        \\" << endl;
  cout << "            --hdg=rand,-45,45 --spd=1,5            \\" << endl;
  cout << "            --ang_min_diff=10 --ang_max_diff=40      " << endl;
  
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pickjoust", "gpl");
  exit(0);
}
