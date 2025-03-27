/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: AdjPath.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "AdjPath.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

AdjPath::AdjPath()
{
  m_output_var = "ADJ_PATH_OUT";
  m_initial_spec = "";
  m_output_prefix = "";
}

//---------------------------------------------------------
// Destructor

AdjPath::~AdjPath()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool AdjPath::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if(key == "SEG_UP_X") {
      m_seg_list.shift_horz(msg.GetDouble());
      Notify(m_output_var, m_output_prefix + m_seg_list.get_spec_pts_label());

    } else if (key == "SEG_UP_Y") {
      m_seg_list.shift_vert(msg.GetDouble());
      Notify(m_output_var, m_output_prefix + m_seg_list.get_spec_pts_label());

    } else if(key == "SEG_ROT") {
      m_seg_list.rotate(msg.GetDouble());
      Notify(m_output_var, m_output_prefix + m_seg_list.get_spec_pts_label());

    } else if (key == "SEG_GROW") {
      m_seg_list.grow_by_amt(msg.GetDouble());
      Notify(m_output_var, m_output_prefix + m_seg_list.get_spec_pts_label());

    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool AdjPath::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool AdjPath::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool AdjPath::OnStartUp()
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
    if(param == "nominal_path") {
      XYSegList new_seglist = string2SegList(value);
      if(new_seglist.size() > 0){
	
	m_seg_list = new_seglist;
	m_initial_spec = value; 
	handled = true; 
      }
    }
    else if(param == "output_var") {
      if (value != ""){
	m_output_var = toupper(value);
	handled = true;
      }
    }
    else if(param == "output_prefix") {
      if (value != ""){
	m_output_prefix = value;
	handled = true;
      }
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  if (m_seg_list.size() == 0) {
    reportUnhandledConfigWarning("Error - Initial Path not set");

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void AdjPath::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
   Register("SEG_UP_X", 0);
   Register("SEG_UP_Y", 0);
   Register("SEG_ROT", 0);
   Register("SEG_GROW", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool AdjPath::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "Initial Spec: " << m_initial_spec <<   endl;
  m_msgs << "Seg List Spec: " << m_seg_list.get_spec()  <<   endl;
  m_msgs << "============================================" << endl;

  
  /*
  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();
  */

  return(true);
}




