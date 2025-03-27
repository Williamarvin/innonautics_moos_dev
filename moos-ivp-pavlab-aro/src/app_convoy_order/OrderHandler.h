/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoyOrderHandler.h                                 */
/*    DATE: June 4th, 2022                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef CONVOY_ORDER_HANDLER_HEADER
#define CONVOY_ORDER_HANDLER_HEADER

#include <string>
#include <map>
#include "ConvoyOrderDetector.h"

class OrderHandler
{
 public:
  OrderHandler();
  ~OrderHandler() {}

  bool handle();
  
  bool setParam(std::string, std::string);
  
protected: // Config vars

  std::string m_file;
  bool        m_verbose;

 protected: // State vars

  unsigned int m_unhandled_lines;
  
  ConvoyOrderDetector m_odetector;
};

#endif


