/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: main.cpp, Cambridge MA                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <string>
#include "MBUtils.h"
#include "ColorParse.h"
#include "KayakEvalEngine.h"


using namespace std;

int main(int argc, char *argv[])
{
  string mission_file;
  string run_command = argv[0];
  KayakEvalEngine KayakEvalEngine;

  for(int i=1; i<argc; i++) {
    string argi = argv[i];
    if((argi=="-v") || (argi=="--version") || (argi=="-version"))
      KayakEvalEngine.showReleaseInfoAndExit();
    else if((argi=="-e") || (argi=="--example") || (argi=="-example"))
      KayakEvalEngine.showExampleConfigAndExit();
    else if((argi == "-h") || (argi == "--help") || (argi=="-help"))
      KayakEvalEngine.showHelpAndExit();
    else if((argi == "-i") || (argi == "--interface"))
      KayakEvalEngine.showInterfaceAndExit();
    else if(strEnds(argi, ".moos") || strEnds(argi, ".moos++"))
      mission_file = argv[i];
    else if(strBegins(argi, "--alias="))
      run_command = argi.substr(8);
    else if(i==2)
      run_command = argi;
  }
  
  if(mission_file == "")
    KayakEvalEngine.showHelpAndExit();

  cout << termColor("green");
  cout << "pKayakEvalEngine launching as " << run_command << endl;
  cout << termColor() << endl;

  KayakEvalEngine.Run(run_command.c_str(), mission_file.c_str());
  
  return(0);
}

