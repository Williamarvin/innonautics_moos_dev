/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PingProfileDetect.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "PingProfileDetect.h"

using namespace std;

//---------------------------------------------------------
// Constructor

PingProfileDetect::PingProfileDetect()
{
  m_get_profile=true;
  m_got_profile=false;
    m_start_ind=90;
    m_end_ind=170;
  m_last_indice=-1;

}

//---------------------------------------------------------
// Destructor

PingProfileDetect::~PingProfileDetect()
{
}

bool PingProfileDetect::getIndex(vector<int> profile)
{
  bool notgot_1=true;
  bool got_max=false;
  bool n=true;
  int check1=0;
    for (int i=0; i<profile.size(); i++){
      if(profile[i]>252&&!got_max){
        got_max=true;
      }
      else if(!got_max){
        check1++;
      }
      if (profile[i]>=252&&notgot_1&&got_max){
        check1++;
      }
      if (profile[i]<252&&notgot_1&&got_max){
        notgot_1=false;
      }
      if(i-check1>2&&n){
        m_start_ind=i;
        n=false;
      }
    }
    n=true;
  notgot_1=true;
  got_max=false;
  check1=profile.size()-1;
    for (int i=profile.size()-1; i>0; i--){
      if(profile[i]>252&&!got_max){
        got_max=true;
      }
      else if(!got_max){
        check1--;
      }
      if (profile[i]>=252&&notgot_1&&got_max){
        check1--;
      }
      if (profile[i]<252&&notgot_1&&got_max){
        notgot_1=false;
      }
      if(check1-i>2&&n){
        m_end_ind=i;
        n=false;
      }
    }
    m_start_ind=90;
    m_end_ind=170;
    return true;
}
//---------------------------------------------------------
// Procedure: OnNewMail

bool PingProfileDetect::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();


      if(key == "DETECTION_CAL"){
        string sval  = msg.GetString(); 
        if(sval=="true"){
          m_get_profile=true;
        }
      }
     else if(key == "PING_PROFILE"){
      string sval  = msg.GetString(); 
      vector<int> val_list;
      for(int i=0;i<199;i++){
        string param1 = biteStringX(sval, ' ');
        val_list.push_back(atoi(param1.c_str()));
      }
      m_val=sval;
      m_ping_profiles.push_back(val_list);
      if(m_get_profile){
        m_got_profile=getIndex(val_list);
        if(m_got_profile){
          m_get_profile=false;
        }
      }
     }
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool PingProfileDetect::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool PingProfileDetect::Iterate()
{
  AppCastingMOOSApp::Iterate();
  int length=m_ping_profiles.size();
 // m_total.clear();
  int total=0;

  if(length>2&&m_got_profile){
    for(int i=m_last_indice+1; i<m_ping_profiles.size(); i++){
      for(int j=0; j<m_ping_profiles[i].size();j++){
        if(j>m_start_ind&&j<m_end_ind){
          total=total+m_ping_profiles[i][j];
        }
      }
      m_total.push_back(total);
      total=0;
    }
    m_last_indice=m_ping_profiles.size()-1;
  }

 length=m_total.size();
 double test_tot=0;
  if (length>5){
    for(int i=length-1; i>(length-6);i--){
      test_tot=test_tot+m_total[i];
    }
    Notify("FIVE_TOTAL",test_tot);
    Notify("ECHO_AVERAGE",test_tot/5);
    if(test_tot/5>1000){
      Notify("DETECTION","true");
    }

  }



  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool PingProfileDetect::OnStartUp()
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
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void PingProfileDetect::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
   Register("PING_PROFILE", 0);
   Register("DETECTION_CAL", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool PingProfileDetect::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "STARTING INDICE" << endl;
  m_msgs <<m_start_ind<<endl;
  m_msgs << "ENDING INDICE" << endl;
  m_msgs <<m_end_ind<<endl;
  m_msgs << "THE CURRENT SUM INDICE" << endl;
  m_msgs <<m_last_indice<<endl;
  m_msgs << "TOTAL VECTOR SIZE" << endl;
  m_msgs <<m_total.size()<<endl;
  m_msgs << "============================================" << endl;

    m_msgs <<m_val<<endl;
  m_msgs <<m_ping_profiles.size()<<endl;
if(m_total.size()>1){
  for(int i=m_total.size()-1; i>m_total.size()-6; i--){
    m_msgs<<m_total[i]<<endl;
  }
}

  return(true);
}




