/************************************************************/
/*    NAME: Blake Cole                                      */
/*    ORGN: MIT                                             */
/*    FILE: Serial.cpp                                      */
/*    DATE: 25 MARCH 2021                                   */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "Serial.h"

#include <stdio.h>    /* Standard input/output definitions */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */

#include <iostream>    /* Print to terminal */
#include <signal.h>    /* Enable signal interrupts */

using namespace std;

#define TRUE 1
#define FALSE 0

volatile int ACTIVE = TRUE;
volatile int WAIT   = TRUE;

// interrupt functions
void alert_exit (int s)  {ACTIVE = FALSE;};
void alert_newmail (int s)  {WAIT = FALSE;};

//---------------------------------------------------------
// Constructor

Serial::Serial()
{ 
  // Default config variables  (overwritten by .moos params)
  m_port = "/dev/ttyACM0";
  m_baud = 38400;
  m_moosvar_rx = "SERIAL_RX_RAW";
  m_moosvar_tx = "SERIAL_TX";

  // Default state variables
  m_fd  = -1;
  m_ok  = false;
  m_buf = "";
  m_rw  = false;
}


//---------------------------------------------------------
// Destructor

Serial::~Serial()
{
  ClosePort();
}


//---------------------------------------------------------
// Procedure: OnConnectToServer

bool Serial::OnConnectToServer()
{
  registerVariables();
  return(true);
}


//---------------------------------------------------------
// Procedure: registerVariables

void Serial::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  
  if (m_rw) {
    Register(m_moosvar_tx, 0);
  }
}


//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Serial::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = toupper(biteStringX(line, '='));
    string value = line;

    // HANDLE PARAMETERS IN .MOOS FILE ---------------------
    bool handled = false;
    if(param == "SERIAL_PORT") {
      m_port = value;
      handled = true;
    }
    else if(param == "SERIAL_BAUD") {
      handled = setUIntOnString(m_baud, value);
    }
    else if(param == "MOOS_RX_VAR") {
      m_moosvar_rx = value;
      handled = true;
    }
    else if(param == "MOOS_TX_VAR") {
      m_moosvar_tx = value;
      handled = true;
    }
    else if(param == "MODE") {
      if (value == "ASC_READ_ONLY") {
	m_rw = false;
	handled = true;
      } else if (value == "READ_WRITE") {
	m_rw = true;
	handled = true;
      }
    }
    
    

    // REPORT UNHANDLED PARAMETERS ------------------------
    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // INITIATE SERIAL PORT CONNECTION ----------------------
  m_fd = OpenPort();
  m_ok = SetOptions();

  if (m_fd==-1 || !m_ok) {
    Notify("EXITED_NORMALLY", "iSerial");
  }

  // DEFINE INTERRUPTS ------------------------------------
  // exit (ctrl+c) (SIGINT)
  struct sigaction sigExitHandler;
  sigExitHandler.sa_handler = alert_exit;
  sigemptyset(&sigExitHandler.sa_mask);
  sigExitHandler.sa_flags = 0;
  #ifndef __APPLE__
    sigExitHandler.sa_restorer = NULL;
  #endif
  sigaction(SIGINT, &sigExitHandler, NULL);

  // newmail in serial (SIGIO)
  struct sigaction sigNewMailHandler;
  sigNewMailHandler.sa_handler = alert_newmail;
  sigemptyset(&sigNewMailHandler.sa_mask);
  sigNewMailHandler.sa_flags = 0;
  #ifndef __APPLE__
    sigNewMailHandler.sa_restorer = NULL;
  #endif
  sigaction(SIGIO, &sigNewMailHandler, NULL);

  // REGISTER VARIABLES ----------------------------------
  registerVariables();
  return(true);
}


//--------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Serial::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // read port when data available (SIGIO)
  if ( m_ok && (ACTIVE==TRUE) && (WAIT==FALSE) ){
    ReadPort();
    AppCastingMOOSApp::PostReport();
    m_buf.clear();
  }

  if (ACTIVE==FALSE){
    ClosePort();
  }

  return(true);
}


//---------------------------------------------------------
// Procedure: OnNewMail

bool Serial::OnNewMail(MOOSMSG_LIST &NewMail)
{

  // Check mail only if we are writing. 
  if (m_rw) {
    MOOSMSG_LIST::iterator p;
    for(p=NewMail.begin(); p!=NewMail.end(); p++) {
      CMOOSMsg &msg = *p;
      string key    = msg.GetKey();
      
      if (key==m_moosvar_tx && msg.IsString() ){
	string sval = msg.GetString();
	WritePort(sval);
	
      } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
	reportRunWarning("Unhandled Mail: " + key);
    }
    
  }
  AppCastingMOOSApp::OnNewMail(NewMail);
  return(true);

  
}


//------------------------------------------------------------
// Procedure: OpenPort()
int Serial::OpenPort (void)
{
  const char * port = m_port.c_str();

  if (m_rw) {
    m_fd = open(port, O_RDWR | O_NOCTTY );
  } else {
    m_fd = open(port, O_RDONLY | O_NOCTTY | O_NONBLOCK);  
  }

  if (m_fd<0){
    perror(port);
    exit(-1);
  }

  // allow the process to receive async signal (SIGIO)
  fcntl(m_fd, F_SETOWN, getpid());
  fcntl(m_fd, F_SETFL, FASYNC);
  
  return(m_fd);
}


//------------------------------------------------------------
// Procedure: ReadPort()
bool Serial::ReadPort (void)
{
  char buf [256];
  int len = read(m_fd, &buf, sizeof(buf));
  if (len<0)
    cout << "Error reading " << strerror(errno) << endl;
  else if (len==0)
    cout << "Buffer empty." << endl;
  else{
    buf[len] = '\0';

    // typecast char to string for appcast
    // remove newline and carriage return (only go to len-2)
    for (int i = 0; i < len-2; i++) {
        m_buf += buf[i];
    }
    
    Notify(m_moosvar_rx, m_buf);
    WAIT = TRUE;
  }
  return(true);
}


//------------------------------------------------------------
// Procedure: ClosePort()
bool Serial::ClosePort (void)
{
  close(m_fd);
  cout << "\n[data logging stopped by user]\n" << endl;
  return(true);
}


//------------------------------------------------------------
// Procedure: SetOptions()
bool Serial::SetOptions (void)
{
  // **************** CONFIGURE SERIAL PORT ***************** //
  struct termios options;

  // copy current attributes to termios object (options)
  tcgetattr(m_fd, &options);

  // I. CONTROL OPTIONS
  // map integer baudrate to speed_t (termios.h)
  speed_t brate;
  switch (m_baud) {
    case 4800:   brate = B4800;   break;
    case 9600:   brate = B9600;   break;
    case 19200:  brate = B19200;  break;
    case 38400:  brate = B38400;  break;
    case 57600:  brate = B57600;  break;
    case 115200: brate = B115200; break;
    default:
      cout << "unsupported baud rate." << endl;
      return (false);
  }
  
  // set baudrate
  cfsetispeed(&options, brate);
  cfsetospeed(&options, brate);

  // enable receiver and set local mode (mandatory)
  options.c_cflag |= (CLOCAL | CREAD);

  // specify protocol: 8 data bits, no parity, 1 stop bit (8N1)
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // harware control
  //options.c_cflag |= CRTSCTS;  // enable
  options.c_cflag &= ~CRTSCTS;   // disable

  // II. LOCAL OPTIONS
  // set canonical input mode (read full lines only)
  options.c_lflag |= ICANON;
  options.c_lflag &= ~(ECHO | ECHONL | IEXTEN);

  // III. INPUT OPTIONS
  // software control
  //options.c_iflag |= (IXON | IXOFF | IXANY);  // enable
  options.c_iflag &= ~(IXON | IXOFF | IXANY);   // disable

  // preserve carriage return
  options.c_iflag &= ~IGNCR;

  // disable parity checks
  options.c_iflag &= ~INPCK;
  
  // disable end of line conversions
  options.c_iflag &= ~(INLCR | ICRNL | IMAXBEL);

  // disable lowercase conversion
  //options.c_iflag &= ~IUCLC;

  // IV. OUTPUT OPTIONS
  // post-processing
  //options.c_oflag |= OPOST; // enable
  options.c_oflag &= ~OPOST;  // disable
  
  // flush buffer and write updated attributes immediately
  tcflush(m_fd, TCIFLUSH);
  tcsetattr(m_fd, TCSANOW, &options);

  return(true);
}


//------------------------------------------------------------
// Procedure: WritePort(const std::string in

bool Serial::WritePort(const std::string in)
{
  char arr[in.length()+1];
  strcpy(arr, in.c_str());
  arr[in.length()] = '\n';  // replace the '\0'
  
  write(m_fd, arr, sizeof(arr));

  return(true);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool Serial::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: Serial.cpp                            " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "RX var:" << m_moosvar_rx << ",  last value: " << m_buf << endl;
  m_msgs << "TX var:" << m_moosvar_tx << endl;
  
  //ACTable actab(4);
  //actab << "Alpha | Bravo | Charlie | Delta";
  //actab.addHeaderLines();
  //actab << "one" << "two" << "three" << "four";
  //m_msgs << actab.getFormattedString();

  return(true);
}




