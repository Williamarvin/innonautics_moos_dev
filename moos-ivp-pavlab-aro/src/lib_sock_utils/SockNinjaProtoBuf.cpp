/*****************************************************************/
/*    NAME: Tyler Paine, built upon Mike B's sockninja           */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: SockNinjaProtoBuf.cpp                                */
/*    DATE: April 26, 2024                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

//#include <arpa/inet.h>
//#include <unistd.h>
//#include <errno.h>
//#include <fcntl.h>
#include <cstring> // memset
#include "MBUtils.h" // intToString, isValidIPAddress
#include "SockNinjaProtoBuf.h"

using namespace std;

#define BUF_SIZE      4096
#define MAX_BUF_SIZE  4096

//---------------------------------------------------------
// Constructor

SockNinjaProtoBuf::SockNinjaProtoBuf(string ninja_type, int port)
{
  // legal type values are client or server
  m_ninja_type  = "server";
  if(ninja_type == "client")
    m_ninja_type = "client";
 
  memset(m_in_buffer, 0, MAX_NINJA_BUF_SIZE);

  m_port        = port;

  m_ninja_state = "unconnected";

  // To guard against users who never retrieve warnings, events,
  // or retractions, a max list size is enforced.
  m_max_vec_size = 100;

  
  // For clients, this is the server we're trying to connect to.
  // For servers, this will hold the IP of the connected client.
  m_ip_addr = "127.0.0.1";

  m_total_msgs_sent = 0;
  m_total_msgs_rcvd = 0;

}

//---------------------------------------------------------
// Procedure: setIPAddr()
//      Note: Intended to be called by user soon after instantiation

bool SockNinjaProtoBuf::setIPAddr(string ip_addr)
{
  // Check basic structure for IPv4 format N.N.N.N,  0<=N<=256
  if(!isValidIPAddress(ip_addr)) {
    string resolved_ip_addr = getIPAddrByName(ip_addr);
    if(!isValidIPAddress(resolved_ip_addr))
      return(false);
    m_ip_addr_name = ip_addr;
    ip_addr = resolved_ip_addr;
  }
  
  // If this is a server, ip address is discovered when the client
  // connects, so we warn that the ip_addr cannot be directly set.
  if(m_ninja_type == "server")
    addWarning("IP Address can only be set for client comms type");
  
  // Once connected, further changes to ipaddr are not allowed.
  if(m_ninja_state != "unconnected")
    return(false);
  
  m_ip_addr = ip_addr;
  return(true);
}

//---------------------------------------------------------
// Procedure: setPortNumber()

bool SockNinjaProtoBuf::setPortNumber(int port)
{
  // Once connected, further changes to port number are not allowed.
  if(m_ninja_state != "unconnected")
    return(false);
  
  if((port <= 0) || (port > 65334)) {
    addWarning("Invalid port number requested:" + intToString(port));
      return(false);
  }

  m_port = port;
  return(true);
}


void SockNinjaProtoBuf::setMaxVecSize(unsigned int amt)
{
  

}

//---------------------------------------------------------
// Procedure: setupConnection()

bool SockNinjaProtoBuf::setupConnection()
{

  return(true);
}

//---------------------------------------------------------
// Procedure: closeSockFDs()
//   Purpose: Close any open file descriptors. Applications 
//            that use SockNinja are encouraged to call this 
//            prior to exiting. 

void SockNinjaProtoBuf::closeSockFDs()
{

}


//---------------------------------------------------------
// Procedure: readFromSock()

bool SockNinjaProtoBuf::readFromSock()
{

  return(true);
}

//---------------------------------------------------------
// Procedure: sendSockMessageString()
//   Purpose: Sends a string over TCP to the attached client.
//   Returns: true if successful or if blank string (which is not sent)
//            false if error on writing to the TCP port

bool SockNinjaProtoBuf::sendSockMessageString(string str)
{
 

  return(true);
}


//---------------------------------------------------------
// Procedure: sendSockMessageDouble()
//   Purpose: Sends a double over TCP to the attached client.
//   Returns: true if successful or if blank string (which is not sent)
//            false if error on writing to the TCP port

bool SockNinjaProtoBuf::sendSockMessageDouble(double val)
{
 

  return(true);
}



//---------------------------------------------------------
// Procedure: getSockMessagesDouble()
//            Returns true if there are messages,
//                    two vectors of keys and vals by reference
///           The length of the vectors is limited by maxVecsize
bool SockNinjaProtoBuf::getSockMessagesDouble(std::vector<std::string> &keys,
					      std::vector<double> &vals)
{


  return(true);  
}



//---------------------------------------------------------
// Procedure: getSockMessagesString()
//            Returns true if there are messages,
//                    two vectors of keys and vals by reference
///           The length of the vectors is limited by maxVecsize
bool SockNinjaProtoBuf::getSockMessagesString(std::vector<std::string> &keys,
					      std::vector<string> &vals)
{


  return(true);  
}



//---------------------------------------------------------
// Procedure: getIPAddrByName()
//   Purpose: Attempt to resolve the given hostname to an IPv4
//            IP address. Typically will look in the local
//            /etc/hosts file first, and then use the DNS if
//            nothing found in /etc/hosts.

string SockNinjaProtoBuf::getIPAddrByName(string host)
{
  struct addrinfo* result;

  int error = getaddrinfo(host.c_str(), 0, 0, &result);
  if(error) {
    string errstr = gai_strerror(error);
    addWarning("Failed to resolve hostname: " + errstr);
    return("");
  }
  else {
    char ipchar[INET_ADDRSTRLEN];
    if(getnameinfo(result->ai_addr, result->ai_addrlen, ipchar,
		   sizeof(ipchar), 0, 0, NI_NUMERICHOST) == 0) {
      host = string(ipchar);
    }
    else 
      addWarning("Unable to parse given host, using " + host);
  }
  return(host);
}


//---------------------------------------------------------
// Procedure: addWarning()

void SockNinjaProtoBuf::addWarning(string warning)
{

}

