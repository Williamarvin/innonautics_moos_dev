/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OptionMarker.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "OptionMarker.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

OptionMarker::OptionMarker()
{
  m_opinion_saturated_limit = 1.0;
  m_marker_circle.set_edge_size(1.0);
  m_marker_circle.setRad(5.0);

  m_post_circ_with_stale_opinion = false;
  m_last_spec_posted = "";

  m_update = true;
  m_post_visuals = true;
  m_posted_new_visuals = false;
  m_vis_cond_var = "SHOW_MARKER";
}

//---------------------------------------------------------
// Destructor

OptionMarker::~OptionMarker()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool OptionMarker::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if(key == "OWN_OPINION_STATE") {
      OpinionRecord new_record;
      bool ok = new_record.setRecordFromMsg(msg.GetString(), msg.GetTime());
      // Is this a blank record?
      bool is_blank = (new_record.getNumberOfOpinions() == 0);

      // save it under these conditions:
      // if it is not (blank and we want to post it)
      // !(is_blank && m_post_circ_with_stale_opinion)
      // and it is ok
      
      m_update = ( (ok) && !(is_blank && m_post_circ_with_stale_opinion) );
      if (m_update){
	m_own_opinion_rec = new_record;
      } else if (!ok){
	reportRunWarning("Unhandled Mail: " + key);
      }
    } else if (key == "NAV_X") {
      m_marker_circle.setX(msg.GetDouble());
    } else if (key == "NAV_Y") {
      m_marker_circle.setY(msg.GetDouble());
    } else if (key == "RESET_OPINIONS"){
      m_marker_circle.set_active(false); 
      Notify("VIEW_CIRCLE", m_marker_circle.get_spec());
      m_marker_circle.set_active(true);
      
      OpinionRecord new_record;
      m_own_opinion_rec = new_record;
    } else if (key == m_vis_cond_var) {
      setBooleanOnString(m_post_visuals, msg.GetString()); 
      
    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool OptionMarker::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool OptionMarker::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Are we posting visuals
  if (!m_post_visuals){
    
    // do we need to clear previous postings?
    if (m_posted_new_visuals){
      m_marker_circle.set_active(false);
      Notify("VIEW_CIRCLE", m_marker_circle.get_spec());
      m_marker_circle.set_active(true); 
      m_posted_new_visuals = false; 
    }

    // bail here
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  

  double dt = MOOSTime() - m_own_opinion_rec.getTime();
  if ((dt < 3.0) || m_post_circ_with_stale_opinion) {
    std::string option_with_strongest_opinion = m_own_opinion_rec.getStrongestOpinion();
    double opinion = 0.0; 
    bool ok_opinion   = tolower(m_own_opinion_rec.getOpinionForOption(option_with_strongest_opinion, opinion));
    bool ok_color_map = (m_option_color_map.count(option_with_strongest_opinion) > 0); 

    std::string color = "grey";
    if (ok_opinion && ok_color_map) {
      // first determine what color to use
      color = m_option_color_map[option_with_strongest_opinion];

      std::vector<double> rgb_vals = colorParse(color); 
      ColorPack new_color_pack(rgb_vals);
      
      // then determine the saturation
      double ratio = opinion / m_opinion_saturated_limit; //not possible to div 0
      if (ratio > 1.0)
	ratio = 1.0;
      if (ratio < 0.0)
	ratio = 0.0;

      // ratio is in the range [0,1], but we need to shift to
      //       the range where zero is no change and 1 is completely
      //       gray.
      ratio = -1.0*ratio + 1.0;
      
      new_color_pack.moregray(ratio);
      
      m_marker_circle.set_color("edge", new_color_pack);

      std::string mkr_spec = m_marker_circle.get_spec();
      bool new_spec = (mkr_spec != m_last_spec_posted);
      bool post_interval_elapsed = ((m_iteration % 100) == 0);
      post_interval_elapsed = false;
      
      if ( new_spec || post_interval_elapsed || m_update){
	Notify("VIEW_CIRCLE", m_marker_circle.get_spec());
	m_last_spec_posted = mkr_spec;
	m_update = false;
	m_posted_new_visuals = true; 
      }
    }
  }
  
  // Do your thing here!
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool OptionMarker::OnStartUp()
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
    if(param == "edge_size") {
      double temp_val = 1.0;
      bool ok = setPosDoubleOnString(temp_val, value);
      if (ok){
	m_marker_circle.set_edge_size(temp_val); 
	handled = true;
      }
    } else if(param == "radius") {
      double temp_val = 1.0;
      bool ok = setPosDoubleOnString(temp_val, value);
      if (ok){
	m_marker_circle.setRad(temp_val); 
	handled = true;
      }
    } else if(param == "opinion_saturated_limit") {
      double temp_val = 1.0;
      handled = setPosDoubleOnString(temp_val, value);
      if (handled)
	m_opinion_saturated_limit = temp_val;
      
    } else if(param == "color_set") {
      handled = handleColorSpec(value);
      
    } else if(param == "post_circ_with_stale_opinion"){
      handled = setBooleanOnString(m_post_circ_with_stale_opinion, value);
    } else if(param == "visibility_cond_var") {
      if (value != ""){
	m_vis_cond_var = toupper(value);
	handled = true; 
      }
    }
    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  m_marker_circle.set_label(m_host_community + "'s opt_circ"); 
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void OptionMarker::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("OWN_OPINION_STATE", 0);
  Register("RESET_OPINIONS",0); 
  Register("NAV_X",0);
  Register("NAV_Y",0);
  Register(m_vis_cond_var, 0); 
}


//------------------------------------------------------------
// Procedure: handleColorSpec(std::string val)
//            EX loiter1:green

bool OptionMarker::handleColorSpec(std::string spec)
{
  string optn_name = tolower(biteStringX(spec, ':'));
  string color = spec;

  if (optn_name == "")
    return(false);
  if (!isColor(color))
    return(false);
  
  m_option_color_map[optn_name] = spec;
  return(true); 
}



//------------------------------------------------------------
// Procedure: buildReport()

bool OptionMarker::buildReport() 
{
  m_msgs << "============================================" << endl;
 
  m_msgs << "                                            " << endl;
  m_msgs << "Color map:                                  " << endl;
  std::map<std::string, std::string>::iterator it;
  for (it = m_option_color_map.begin(); it != m_option_color_map.end(); it++){
    m_msgs << " * " << it->first << ":" << it->second << endl; 
  }
  m_msgs << "m_post_circ_with_stale_opinion = " << std::to_string(m_post_circ_with_stale_opinion) << endl;
  m_msgs << " Last marker spec: " << m_last_spec_posted << endl;
  
  //ACTable actab(4);
  //actab << "Alpha | Bravo | Charlie | Delta";
  //actab.addHeaderLines();
  //actab << "one" << "two" << "three" << "four";
  //m_msgs << actab.getFormattedString();

  return(true);
}




