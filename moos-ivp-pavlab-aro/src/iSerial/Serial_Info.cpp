/****************************************************************/
/*   NAME: Blake Cole                                           */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: Serial_Info.cpp                                      */
/*   DATE: 5 APRIL 2021                                         */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "Serial_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("The iSerial application is used for reading full lines of data  ");
  blk("from a serial buffer, given a user-defined SERIAL PORT and      ");
  blk("BAUD RATE.  By default, iSerial operates in asynchronous,       ");
  blk("non-blocking, canonical mode.                                   ");
  blk("                                                                ");
  blk("ASYNCHRONOUS means that the read() function is only called when ");
  blk("data is present in the serial buffer.                           ");
  blk("                                                                ");
  blk("NON-BLOCKING means that the blocking read() function returns    ");
  blk("immediately, regardless of the number of characters present in  ");
  blk("serial buffer, or the time elapsed since read() was called.     ");
  blk("                                                                ");
  blk("CANONICAL MODE means that only full lines are returned by the   ");
  blk("read() function.  Lines are delineated by \r\n.                 ");
  blk("                                                                ");
  blk("Additionally, the app can be configured to both read from and   ");
  blk("write to the serial buffer using the MODE parameter. However,   ");
  blk("app would block on read().  (To use a double negative, the app  ");
  blk("is not non-blocking.)                                           ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iSerial file.moos [OPTIONS]                              ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iSerial with the given process name                ");
  blk("      rather than iSerial.                                      ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iSerial.                   ");
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
  blu("iSerial Example MOOS Configuration                              ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iSerial                                         ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  SERIAL_PORT   = /dev/ttyACM0       // (default)               ");
  blk("  SERIAL_BAUD   = 38400              // (default)               ");
  blk("  MOOS_RX_VAR   = SERIAL_RX_RAW      // (default)               ");
  blk("  MOOS_TX_VAR   = SERIAL_TX          // (default)               ");
  blk("                                                                ");
  blk("  MODE          = ASC_READ_ONLY      // (ASC_READ_ONLY)         ");
  blk("                                     //  or READ_WRITE          ");
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
  blu("iSerial INTERFACE                                               ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  SERIAL_TX     (Only if MODE = READ_WRITE)                     ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  SERIAL_RX_RAW = \"!AIVDM,1,1,,A,35Ngd70P@hrrhc6H?Cnk7BBJ01n@,0*4C\"");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iSerial", "gpl");
  exit(0);
}

