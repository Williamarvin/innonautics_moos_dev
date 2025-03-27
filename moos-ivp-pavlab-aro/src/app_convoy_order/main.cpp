/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: main.cpp                                             */
/*    DATE: June 4th, 2022                                       */
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
#include "OrderHandler.h"

using namespace std;

void showHelpAndExit();

//--------------------------------------------------------
// Procedure: main

int main(int argc, char *argv[])
{
  OrderHandler handler;

  bool handled = false;
  for(int i=1; i<argc; i++) {
    string argi = argv[i];
    if((argi=="-h") || (argi == "--help") || (argi=="-help"))
      showHelpAndExit();
    else if((argi=="-v") || (argi=="--version") || (argi=="-version")) {
      showReleaseInfo("convoy_eval", "gpl");
      return(0);
    }
    else if(argi == "--verbose")
      handled = handler.setParam("verbose", "true");
    else if(strBegins(argi, "--file="))
      handled = handler.setParam("file", argi.substr(7));
    else if(strEnds(argi, ".txt")) 
      handled = handler.setParam("file", argi);
    if(!handled) {
      cout << "Unhandled command line argument: " << argi << endl;
      cout << "Use --help for usage. Exiting.   " << endl;
      exit(1);
    }
  }

  handler.handle();
  exit(0);
}
  
//------------------------------------------------------------
// Procedure: showHelpAndExit()  

void showHelpAndExit()
{
  cout << "Usage: " << endl;
  cout << "  convoy_order file.txt [OPTIONS]                          " << endl;
  cout << "                                                           " << endl;
  cout << "Synopsis:                                                  " << endl;
  cout << "  Given a file of orderings, determine the convoy.         " << endl;
  cout << "                                                           " << endl;
  cout << "Standard Arguments:                                        " << endl;
  cout << "  file.txt - The input file.                               " << endl;
  cout << "                                                           " << endl;
  cout << "Options:                                                   " << endl;
  cout << "  -h,--help             Displays this help message         " << endl;
  cout << "  -v,--version          Displays current release version   " << endl;
  cout << "  --verbose             Displays verbose output            " << endl;
  cout << "                                                           " << endl;
  cout << "  --file=<val>          A file of pairins                  " << endl;
  cout << "                                                           " << endl;
  cout << "Further Notes:                                             " << endl;
  cout << "  (1) Order of arguments is irrelevant.                    " << endl;
  cout << endl;
  exit(0);
}


