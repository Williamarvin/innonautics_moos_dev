/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: TestPost.cpp                                    */
/*    DATE: June 24th, 2022                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "TestPost.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

TestPost::TestPost()
{
  m_posting_len = 27;
}


//---------------------------------------------------------
// Procedure: OnNewMail()

bool TestPost::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    double dval   = msg.GetDouble();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if(key == "POSTING_LEN") 
      m_posting_len = (unsigned int)(dval);
    
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool TestPost::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool TestPost::Iterate()
{
  AppCastingMOOSApp::Iterate();

  string posting = genPosting(m_posting_len);
  Notify("TEST_POST", posting);
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool TestPost::OnStartUp()
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
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "posting_len") {
      handled = setUIntOnString(m_posting_len, value);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void TestPost::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("POSTING_LEN", 0);
}


//------------------------------------------------------------
// Procedure: genPosting()

string TestPost::genPosting(unsigned int amt) 
{
  string str;
  for(unsigned int i=0; i<amt; i++) {
    int index = i % 52;
    char c;
    if(index < 26)
      c = 65 + index;
    else
      c = 97 - 26 + index;
    str += c;
  }

  str = "LEN=" + uintToString(amt) + ":" + str;

  return(str);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool TestPost::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "PostingLen: " << uintToString(m_posting_len)  << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




