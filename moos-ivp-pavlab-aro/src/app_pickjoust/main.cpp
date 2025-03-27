/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: main.cpp                                             */
/*    DATE: Oct 28th 2019                                        */
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

#include <iostream>
#include "MBUtils.h"
#include "PickJoust.h"
#include "PickJoust_Info.h"

using namespace std;

int main(int argc, char *argv[])
{
  PickJoust pickj;

  string arg_summary = argv[0];

  int  file_index = 1;
  
  if(argc == 1)
    showHelpAndExit();

  // Pass One over the command line args
  for(int i=1; i<argc; i++) {
    string argi = argv[i];
    arg_summary += " " + argi;

    bool handled = true;
    if((argi=="-v") || (argi=="--version") || (argi=="-version"))
      showReleaseInfoAndExit();
    else if((argi=="-h") || (argi == "--help") || (argi=="-help"))
      showHelpAndExit();
    else if((argi=="-f") || (argi == "--force") || (argi=="-force"))
      pickj.setFileOverWrite(true);
    else if(argi=="--verbose")
      pickj.setVerbose(true);
    
    else if(strBegins(argi, "--file="))
      handled = pickj.setOutputFile(argi.substr(7));
    else if(strBegins(argi, "--amt="))
      handled = pickj.setPickAmt(argi.substr(6));
    else if(strBegins(argi, "--circle="))
      handled = pickj.setCircle(argi.substr(9));
    else if(strBegins(argi, "--ang_min_diff="))
      handled = pickj.setAngMinDiff(argi.substr(15));
    else if(strBegins(argi, "--ang_max_diff="))
      handled = pickj.setAngMaxDiff(argi.substr(15));
    else if(strBegins(argi, "--maxtries="))
      handled = pickj.setMaxTries(argi.substr(11));
    else if(strBegins(argi, "--spd="))
      handled = pickj.setSpdConfig(argi.substr(6));
    else if(strBegins(argi, "--ssnap="))
      handled = pickj.setSpdSnap(argi.substr(8));
    else if(strBegins(argi, "--psnap="))
      handled = pickj.setPtSnap(argi.substr(8));
    else if(argi=="--reuse")
      pickj.setReuse();
    else if(argi=="--hdrs")
      pickj.enableHeaders();
    else
      handled = false;
      
    if(!handled) {
      cout << "pickjoust: Unhandled arg: " << argi << endl;
      return(0);
    }
  }

  pickj.setArgSummary(arg_summary);
  bool success = pickj.pick();
  if(!success)
    return(1);

  pickj.printChoices();
  
  return(0);
}
