/*****************************************************************/
/*    NAME: Ray Turrisi                                          */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: main.cpp                                             */
/*    DATE: June 20th, 2023                                       */
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
#include "EvalEngine.h"
#include "EvalEnginePostProcessor.h"
#include "std_ee_kernels.h"

using namespace std;

void showHelpAndExit();

//--------------------------------------------------------
// Procedure: main

int main(int argc, char *argv[])
{

  EvalEngine evalEngine;
  evalEngine.insertKernel(new xSurfaceOdometryKernel());
  evalEngine.insertKernel(new xCounterKernel("xCounter1", "_1"));
  evalEngine.insertKernel(new xCounterKernel("xCounter2", "_2"));
  evalEngine.insertKernel(new xStateTimer());
  
  std::string appName = argv[0];
  EvalEnginePostProcessor handler(&evalEngine, appName);

  bool handled = false;
  
  for(int i=1; i<argc; i++) {
    string argi = argv[i];
    if((argi=="-h") || (argi == "--help") || (argi=="-help"))
      showHelpAndExit();
    else if(argi == "--verbose")
      handled = handler.setParam("verbose", "false");
    else if(argi == "--rewrite")
      handled = handler.setParam("rewrite", "true");
    else if(strBegins(argi, "--tag="))
      handled = handler.setParam("tag", argi.substr(6));
    else if(argi == "--quiet")
      handled = handler.setParam("quiet", "true");
    else if(strBegins(argi,"--pname="))
      handled = handler.setParam("process_name", argi.substr(8));
    else if(strEnds(argi, ".moos")) 
      handled = handler.setParam("moos_file", argi);
    else if(isValidDirectory(argi)) {
      //We check to see if the target directory is valid (is a directory and contains and alog file), and then also retrieve/cache the alog file
      handled = handler.setParam("target_directory", argi);
    }
    if(!handled) {
      cout << "Unhandled command line argument: " << argi << endl;
      cout << "Use --help for usage. Exiting.   " << endl;
      exit(1);
    }
  }
  handler.process();
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


