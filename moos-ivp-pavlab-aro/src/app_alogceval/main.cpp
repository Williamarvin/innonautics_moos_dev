/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: main.cpp                                             */
/*    DATE: Aug 24th, 2021                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <string>
#include <cstdlib>
#include <iostream>
#include "MBUtils.h"
#include "ReleaseInfo.h"
#include "LogEvalHandler.h"

using namespace std;

void showHelpAndExit();

//--------------------------------------------------------
// Procedure: main

int main(int argc, char *argv[])
{
  LogEvalHandler handler;

  bool handled = false;
  for(int i=1; i<argc; i++) {
    string argi = argv[i];
    if((argi=="-h") || (argi == "--help") || (argi=="-help"))
      showHelpAndExit();
    else if((argi=="-v") || (argi=="--version") || (argi=="-version")) {
      showReleaseInfo("alogbin", "gpl");
      return(0);
    }
    else if(strEnds(argi, ".alog"))
      handled = handler.setALogFile(argi);
    else if(strBegins(argi, "--file="))
      handled = handler.setALogFile(argi.substr(7));
    else if(strBegins(argi, "--ott="))
      handled = handler.setParam("on_tail_thresh", argi.substr(6));
    else if(strBegins(argi, "--at="))
      handled = handler.setParam("alignment_thresh", argi.substr(5));
    else if(strBegins(argi, "--tt="))
      handled = handler.setParam("tracking_thresh", argi.substr(5));
    else if(strBegins(argi, "--rst="))
      handled = handler.setParam("rng_switch_thresh", argi.substr(5));
    else if(strBegins(argi, "--tes="))
      handled = handler.setParam("track_err_snap", argi.substr(6));
    else if(strBegins(argi, "--rep="))
      handled = handler.setParam("report", argi.substr(6));
    else if(argi ==  "-r1")
      handled = handler.setParam("report", "appcast");
    else if(argi ==  "-r2")
      handled = handler.setParam("report", "track_err_bins");
    else if(argi=="--verbose")
      handled = handler.setParam("verbose", "true");

    if(!handled) {
      cout << "Unhandled command line argument: " << argi << endl;
      cout << "Use --help for usage. Exiting.   " << endl;
      exit(1);
    }
  }

  handler.handle();
  handler.makeReports();
  exit(0);
}
  
//------------------------------------------------------------
// Procedure: showHelpAndExit()  

void showHelpAndExit()
{
  cout << "Usage: " << endl;
  cout << "  alogceval file.log [OPTIONS]                             " << endl;
  cout << "                                                           " << endl;
  cout << "Synopsis:                                                  " << endl;
  cout << "  Given an alog file, evaluate convoy metrics.             " << endl;
  cout << "                                                           " << endl;
  cout << "Standard Arguments:                                        " << endl;
  cout << "  file.alog - The input alogfile.                          " << endl;
  cout << "                                                           " << endl;
  cout << "Options:                                                   " << endl;
  cout << "  -h,--help             Displays this help message         " << endl;
  cout << "  -v,--version          Displays current release version   " << endl;
  cout << "                                                           " << endl;
  cout << "  --ott=<val>           Set on_tail_thresh (default=10)    " << endl;
  cout << "  --at=<val>            Set align_thresh (default=60)      " << endl;
  cout << "  --tt=<val>            Set tracking_thresh (default=3)    " << endl;
  cout << "  --tes=<val>           Set track_err_snap (default=0.1)   " << endl;
  cout << "                                                           " << endl;
  cout << "  --rep=appcast         Show the final Appcast output      " << endl;
  cout << "  --rep=track_err_bins  Show the track error by bins       " << endl;
  cout << "                                                           " << endl;
  cout << "  -r1                   Same as --rep=appcast              " << endl;
  cout << "  -r2                   same as --rep=track_err_bins       " << endl;
  cout << "                                                           " << endl;
  cout << "Further Notes:                                             " << endl;
  cout << "  (1) Order of arguments is irrelevant.                    " << endl;
  cout << endl;
  exit(0);
}


