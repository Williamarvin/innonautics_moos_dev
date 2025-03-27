/*****************************************************************/
/*    NAME: Tyler Paine, built upon Mike B's sockninja           */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: SockNinjaProtoBuf.h                                  */
/*    DATE: April 26, 2024                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef SOCK_NINJA_PROTOBUF_HEADER
#define SOCK_NINJA_PROTOBUF_HEADER

//#include <sys/socket.h>
#include <netdb.h>
#include <string>
#include <list> 
#include <map> 

#define MAX_NINJA_BUF_SIZE 16384

class SockNinjaProtoBuf {
public:
  SockNinjaProtoBuf(std::string ntype="server", int port=50051);
  ~SockNinjaProtoBuf() {closeSockFDs();}
 
  bool  setIPAddr(std::string); // ok
  bool  setPortNumber(int);     // ok

  void  setMaxVecSize(unsigned int);
  bool  setupConnection();
  void  closeSockFDs();

  bool  sendSockMessageString(std::string str);
  bool  sendSockMessageDouble(double val);
  
  bool  getSockMessagesDouble(std::vector<std::string> &keys,
			      std::vector<double> &vals);
  bool  getSockMessagesString(std::vector<std::string> &keys,
			      std::vector<std::string> &vals);
  
protected:
  bool  readFromSock();
  
  std::string getIPAddrByName(std::string);

  void addWarning(std::string warning); 
  
protected:
  std::string  m_ninja_type;
  std::string  m_ninja_state;
  
  unsigned int m_max_vec_size;
  
  char         m_in_buffer[MAX_NINJA_BUF_SIZE];;
  int          m_port;
  
  std::string  m_ip_addr; 
  std::string  m_ip_addr_name; 
    
  unsigned int m_total_msgs_sent;
  unsigned int m_total_msgs_rcvd;

};

#endif


