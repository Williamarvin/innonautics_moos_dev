/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: UtilVectorMarker.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "UtilVectorMarker.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

UtilVectorMarker::UtilVectorMarker()
{
  m_nav_x = 0.0;
  m_nav_y = 0.0;

  m_head_size = -1.0;
  m_max_mag   = 40;
  m_vect_length_scale = 10;

  m_vname = "";
  m_vis_cond_var = "VECTOR_VIS";

  m_post_visuals = true;
  m_posted_new_visuals = false; 
}

//---------------------------------------------------------
// Destructor

UtilVectorMarker::~UtilVectorMarker()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool UtilVectorMarker::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if(key == "NAV_X") {
      m_nav_x = msg.GetDouble();
    } else if (key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
    } else if (m_input_vars.count(key) > 0){
      handleNewInput(key, msg.GetDouble());
    } else if (key == m_vis_cond_var) {
      setBooleanOnString(m_post_visuals, msg.GetString()); 
    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool UtilVectorMarker::OnConnectToServer()
{
  //registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool UtilVectorMarker::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Are we posting visuals
  if (!m_post_visuals){
    
    // do we need to clear previous postings?
    if (m_posted_new_visuals){
        std::map<std::string, XYVector>::iterator it;
	for (it = m_vec_map.begin(); it != m_vec_map.end(); it++){
	  it->second.set_active(false); 
	  Notify("VIEW_VECTOR", it->second.get_spec());
	  it->second.set_active(true);
	}
	m_posted_new_visuals = false; 
    }

    // bail here
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // Update the angle of the vector based on ownship position
  // and the xy point and the location origin,
  // Then post it
  std::map<std::string, XYVector>::iterator it;
  for (it = m_vec_map.begin(); it != m_vec_map.end(); it++){
    std::string var_name = it->first;

    double angle = relAng(m_nav_x, m_nav_y, m_point_map[var_name].get_vx(), m_point_map[var_name].get_vy());
    double mag = it->second.mag();
    it->second.setVectorMA(mag, angle);

    it->second.setPosition(m_nav_x, m_nav_y);
    it->second.setHeadSize(m_head_size); 
    
    
    Notify("VIEW_VECTOR", it->second.get_spec());
    m_posted_new_visuals = true; 
  }

  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool UtilVectorMarker::OnStartUp()
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
    if(param == "head_size") {
      handled = setDoubleOnString(m_head_size, value);
    } else if(param == "max_magnitude") {
      handled = setDoubleOnString(m_max_mag, value);
    } else if(param == "vect_length_scale") {
      handled = setDoubleOnString(m_vect_length_scale, value);
    } else if(param == "vector_set") {
      handled = handleVectorSet(value); 
    } else if(param == "visibility_cond_var") {
      if (value != ""){
	m_vis_cond_var = toupper(value);
	handled = true; 
      }
    }


    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void UtilVectorMarker::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
   Register("NAV_X", 0);
   Register("NAV_Y", 0);
   std::set<std::string>::iterator it;
   for (it = m_input_vars.begin(); it != m_input_vars.end(); it++){
     Register( *it, 0);
   }
   Register(m_vis_cond_var, 0); 
}


//---------------------------------------------------------
// Procedure: handleVectorSet()
bool UtilVectorMarker::handleVectorSet(std::string spec)
{
  std::string var = "";
  std::string color = "";
  XYPoint point;
  
  std::vector<std::string> spec_vec = parseString(spec,':');
  for (int i = 0; i < spec_vec.size(); i++){
    std::string line = spec_vec[i];
    string param = tolower(biteStringX(line, '='));
    string value = line;
    
    if (param == "var"){
      var = toupper(value);
      Notify("TMP_VAR = ", var);
     
    } else if (param == "color"){
      color = value;
      Notify("TMP_Color = ", color);

    } else if (param == "point_to"){
      point = string2Point(value);
      Notify("TMP_pre_point", value); 
      Notify("TMP_Point = ", point.get_spec());
    }

  }

  if ((var == "") || (!isColor(color)) || (!point.valid()))
    return(false);

  // Ok all good, update the records.
  // From now on we assume these all have
  // the same VAR names.  This could be
  // done with a custom data structure,
  m_input_vars.insert(toupper(var));
  
  XYVector vector;
  vector.set_color("edge",color);
  vector.set_color("fill",color);
  vector.set_label(m_host_community + "'s util vect for opt " + tolower(var));
  m_vec_map[toupper(var)] = vector;

  m_point_map[toupper(var)] = point;
  

  return(true); 
}

//------------------------------------------------------------
// Procedure: buildReport()
void UtilVectorMarker::handleNewInput(std::string key, double val)
{
  double vect_length = val * m_vect_length_scale;
  if (vect_length > m_max_mag)
    vect_length = m_max_mag;

  m_vec_map[toupper(key)].setVectorMA(vect_length, 0.0); 

}

//------------------------------------------------------------
// Procedure: buildReport()

bool UtilVectorMarker::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




