/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: SockComms.h                                          */
/*    DATE: March 22nd 2020                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef SOCK_COMMS_HEADER
#define SOCK_COMMS_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "SockNinja.h"

class SockComms : public AppCastingMOOSApp
{
 public:
  SockComms();
  ~SockComms() {};

 protected: // Standard MOOSApp functions to overload  
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  
 protected: // Standard AppCastingMOOSApp function to overload 
  bool buildReport();
  void reportWarningsEvents();
  
 protected:
  void handleMailPostSockMsg(std::string);
  void sendMessagesToSocket();
  void readMessagesFromSocket();
  void registerVariables();

 private: // Config variable
  std::string m_message;
  
 private: // State variables
  
  SockNinja   m_ninja;
  
  std::string m_latest_rx_msg;
  std::string m_latest_tx_msg;
  std::list<std::string> m_sock_msgs;
};

#endif 


