/****************************************************************/
/*   NAME:                                              */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: HydroLinkArduinoBridge_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "HydroLinkArduinoBridge_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"


using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The iHydroLinkArduinoBridge application is used for               ");
  blk("                                                                ");
  blk("                                                                ");
  blk("                                                                ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iHydroLinkArduinoBridge file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iHydroLinkArduinoBridge with the given process name         ");
  blk("      rather than iHydroLinkArduinoBridge.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iHydroLinkArduinoBridge.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit(HydroLinkArduinoBridge &hydrolink)
{
  std::cout << "//------------------------------------------------" << std::endl;
  std::cout << "//iHydroLink config block" << std::endl;
  std::cout << "ProcessConfig = iHydroLink" << std::endl;
  std::cout << "{" << std::endl;
  std::cout << "\tAppTick   = 4" << std::endl;
  std::cout << "\tCommsTick = 4" << std::endl;
  
  std::unordered_map<std::string, std::string> params = hydrolink.get_params();
  uint64_t longest_name = 0;
  std::unordered_map<std::string, std::string>::iterator param_it;
  for(param_it = params.begin(); param_it != params.end(); ++param_it)
      if(param_it->first.size() > longest_name) longest_name = param_it->first.size();

  for(param_it = params.begin(); param_it != params.end(); ++param_it) {
    std::cout << "\t    " << std::right << std::setw(longest_name) << param_it->first << " = " << param_it->second << std::endl;
  }
  
  std::cout << "}" << std::endl;
  // iHydroLink config block
  // blk("                                                                ");
  // blu("=============================================================== ");
  // blu("iHydroLink Example MOOS Configuration                   ");
  // blu("=============================================================== ");
  // blk("                                                                ");
  // blk("ProcessConfig = iHydroLink                              ");
  // blk("{                                                               ");
  // blk("  AppTick   = 4                                                 ");
  // blk("  CommsTick = 4                                                 ");
  // blk("                                                                ");
  // blk("}                                                               ");
  // blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iHydroLink INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_MESSAGE = src_node=alpha,dest_node=bravo,var_name=FOO,   ");
  blk("                 string_val=BAR                                 ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  Publications are determined by the node message content.      ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iHydroLink", "gpl");
  exit(0);
}

