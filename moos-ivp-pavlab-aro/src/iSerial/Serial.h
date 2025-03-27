/************************************************************/
/*    NAME: Blake Cole                                      */
/*    ORGN: MIT                                             */
/*    FILE: Serial.h                                        */
/*    DATE: 5 APRIL 2021                                    */
/************************************************************/

#ifndef Serial_HEADER
#define Serial_HEADER

#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class Serial : public AppCastingMOOSApp
{
 public:
   Serial();
   ~Serial();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();
   void registerVariables();

 protected:  // serial port functions
  int   OpenPort (void);
  bool  SetOptions (void);
  //bool  SetInterrupt (int CODE, void (*func)(int s));
  bool  ReadPort (void);
  bool  ClosePort (void);
  bool  WritePort(const std::string in);

 private: // Configuration variables
  std::string     m_port;
  unsigned int    m_baud;
  std::string     m_moosvar_rx;
  std::string     m_moosvar_tx;

  bool            m_rw;  // allow for read and write

 private: // State variables
  int   m_fd;  // serial file id
  bool  m_ok;  // port config ok
  std::string  m_buf; // serial buffer
};

#endif 
