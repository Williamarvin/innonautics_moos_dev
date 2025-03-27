/************************************************************/
/*    NAME: Blake Cole                                      */
/*    ORGN: MIT                                             */
/*    FILE: serial_ais.cpp                                  */
/*    DATE: 30 MARCH 2021                                   */
/************************************************************/

// SERIAL_AIS reads an asychronous canonical serial connection

// References:
//   (1) "Serial Port Programming for POSIX Operating Systems"
//       <https://www.c-program.com/pdf/serialPort_Programming_c.pdf>
//
//   (2) "Serial Programming HOWTO"
//       <https://tldp.org/HOWTO/Serial-Programming-HOWTO/>

#include <stdio.h>    /* Standard input/output definitions */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */

#include <iostream>    /* Print to terminal */
#include <signal.h>    /* Enable signal interrupts */

using namespace std;

#define BAUDRATE B38400 // for dAISyHAT
//#define BAUDRATE B9600    // for Arduino

#define PORT "/dev/serial0" // for dAISyHAT
//#define PORT "/dev/ttyACM0" // for Arduino

#define TRUE 1
#define FALSE 0

volatile int ACTIVE = TRUE;
volatile int WAIT = TRUE;

// declare serial port functions
int open_port (void);
int set_options (int fd);
int read_port (int fd);
int close_port (int fd);

// define interrupt functions
void interrupt (int s) {ACTIVE = FALSE;};
void newmail (int s) {WAIT = FALSE;};


int main ()
{
  // open serial port
  int fd = open_port();
  cout << "port open." << endl;

  // set serial port options
  set_options(fd);
  cout << "options set." << endl;

  // define ctrl+c interrupt
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = interrupt;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigIntHandler.sa_restorer = NULL;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // define asynchronous i/o interrupt
  struct sigaction sigNewMailHandler;
  sigNewMailHandler.sa_handler = newmail;
  sigemptyset(&sigNewMailHandler.sa_mask);
  sigNewMailHandler.sa_flags = 0;
  sigNewMailHandler.sa_restorer = NULL;
  sigaction(SIGIO, &sigNewMailHandler, NULL);

  // enter infinite read loop
  unsigned int millisec = 1000;
  while (ACTIVE){
    usleep(100 * millisec); //sleep for 0.1 second

    // read port when data available (SIGIO)
    if (WAIT==FALSE){
      read_port(fd);
    }

    // close port when when user halts program (SIGINT) 
    if (ACTIVE==FALSE){
      close_port(fd);
      break;
    }
  }
  return(0);
}


int open_port (void)
{
  int fd;  // file descriptor for serial port

  fd = open(PORT, O_RDONLY | O_NOCTTY | O_NONBLOCK);

  if (fd<0){
    perror(PORT);
    exit(-1);
  }

  // allow the process to receive async signal (SIGIO)
  fcntl(fd, F_SETOWN, getpid());

  // F_SETFL=FNDELAY: return 0 if no characters are available on the port
  // F_SETFL=0      : block until characters arrive
  fcntl(fd, F_SETFL, FASYNC);
  
  return(fd);
}


int set_options (int fd)
{
  // **************** CONFIGURE SERIAL PORT ***************** //
  struct termios options;

  // copy current attributes to termios object (options)
  tcgetattr(fd, &options);

  // I. CONTROL OPTIONS
  // set baudrate
  cfsetispeed(&options, BAUDRATE);
  cfsetospeed(&options, BAUDRATE);

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
  options.c_iflag &= ~IUCLC;

  // IV. OUTPUT OPTIONS
  // post-processing
  //options.c_oflag |= OPOST; // enable
  options.c_oflag &= ~OPOST;  // disable
  
  // flush buffer and write updated attributes immediately
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &options);

  return(0);
}


int read_port(int fd){
  char buf [256];
  int len = read(fd, &buf, sizeof(buf));
  if (len<0)
    cout << "Error reading " << strerror(errno) << endl;
  else if (len==0)
    cout << "Buffer empty." << endl;
  else{
    buf[len] = '\0';
    cout << buf;
    WAIT = TRUE;
  }
  return(0);
}


int close_port (int fd){
  close(fd);
  cout << "\n[data logging stopped by user]\n" << endl;
  return(0);
}


/* ----- ALT METHOD FOR NONBLOCKING QUERY ------*/
// #include <sys/ioctl.h>
// int bytes;
// ioctl(fd, FIONREAD, &bytes);
// if (bytes > 0){ read(fd, &buf, sizeof(buf));}
/* ---------------------------------------------*/
