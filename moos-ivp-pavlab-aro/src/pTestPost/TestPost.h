/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: TestPost.h                                      */
/*    DATE: June 24th, 2022                                 */
/************************************************************/

#ifndef TEST_POST_HEADER
#define TEST_POST_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class TestPost : public AppCastingMOOSApp
{
 public:
   TestPost();
  ~TestPost() {};

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
  std::string genPosting(unsigned int);
  bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables
  unsigned int m_posting_len;
  
 private: // State variables
};

#endif 
