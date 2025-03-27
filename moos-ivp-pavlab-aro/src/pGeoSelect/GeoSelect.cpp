/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GeoSelect.cpp                                   */
/*    DATE: July 7th, 2022                                  */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "HashUtils.h"
#include "ACTable.h"
#include "GeoSelect.h"
#include "MacroUtils.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsSegl.h"
#include "VarDataPairUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

GeoSelect::GeoSelect()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GeoSelect::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key  = msg.GetKey();
    string sval = msg.GetString(); 

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    bool handled = false;
    if(key == "GSPOLY_REQ") 
      handled = handleMailGSPolyReq(sval);
    else if(key == "GSPOLY_OFF") 
      handled = handleMailGSPolyOff(sval);
    else if(key == "GSPOLY_ACT") 
      handled = handleMailGSPolyAct(sval);
    
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GeoSelect::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool GeoSelect::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool GeoSelect::OnStartUp()
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
    if(param == "poly")
      handled = handleConfigPolygon(value);
    else if(param == "segl") 
      handled = handleConfigSegList(value);
    else if(param == "poly_flag") 
      handled = addVarDataPairOnString(m_poly_flags, value);

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void GeoSelect::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("GSPOLY_REQ", 0);
  Register("GSPOLY_OFF", 0);
  Register("GSPOLY_ACT", 0);
}


//---------------------------------------------------------
// Procedure: handleConfigPolygon()

bool GeoSelect::handleConfigPolygon(string str)
{
  XYPolygon poly = string2Poly(str);
  if(!poly.is_convex())
    return(false);

  string label = poly.get_label();
  if(label == "")
    label = hashAlphaNum();

  m_map_polys[label] = poly;
  return(true);
}

//---------------------------------------------------------
// Procedure: handleConfigSegList()

bool GeoSelect::handleConfigSegList(string str)
{
  XYSegList segl = string2SegList(str);
  if(segl.size() == 0)
    return(false);

  string label = segl.get_label();
  if(label == "")
    label = hashAlphaNum();

  m_map_segls[label] = segl;
  return(true);
}

//---------------------------------------------------------
// Procedure: handleMailGSPolyReq()

bool GeoSelect::handleMailGSPolyReq(string label)
{
  if(m_map_polys.count(label) == 0)
    return(false);

  if(m_active_poly != "")
    erasePoly(m_map_polys[m_active_poly]);
  
  m_active_poly = label;
  drawPoly(m_map_polys[label]);
  
  return(true);
}

//---------------------------------------------------------
// Procedure: handleMailGSPolyOff()

bool GeoSelect::handleMailGSPolyOff(string label)
{
  if(m_map_polys.count(label) == 0)
    return(false);
  
  erasePoly(m_map_polys[label]);
  return(true);
}

//---------------------------------------------------------
// Procedure: handleMailGSPolyAct()

bool GeoSelect::handleMailGSPolyAct(string label)
{
  if((label != "true") && (m_map_polys.count(label) == 0))
    return(false);

  reportEvent("posting poly_flags");
  postFlags(m_poly_flags);
  
  return(true);
}


//---------------------------------------------------------
// Procedure: drawPoly()

void GeoSelect::drawPoly(XYPolygon poly)
{
  if(!poly.is_convex())
    return;
  string spec = poly.get_spec();
  Notify("VIEW_POLYGON", spec);
}

//---------------------------------------------------------
// Procedure: erasePoly()

void GeoSelect::erasePoly(XYPolygon poly)
{
  string spec = poly.get_spec_inactive();
  Notify("VIEW_POLYGON", spec);
}


//------------------------------------------------------------
// Procedure: postFlags() 

void GeoSelect::postFlags(const vector<VarDataPair>& flags)
{
  for(unsigned int i=0; i<flags.size(); i++) {
    VarDataPair pair = flags[i];
    string moosvar = pair.get_var();

    // If posting is a double, just post. No macro expansion
    if(!pair.is_string()) {
      double dval = pair.get_ddata();
      Notify(moosvar, dval);
    }
    // Otherwise if string posting, handle macro expansion  
    else {
      string sval = pair.get_sdata();

      if(m_map_polys.count(m_active_poly)) {
	string spec = m_map_polys[m_active_poly].get_spec();
	sval = macroExpand(sval, "POLY", spec);
	sval = macroExpand(sval, "PLABEL", m_active_poly);
      }
      
      Notify(moosvar, sval);
    }
  }
}




//------------------------------------------------------------
// Procedure: buildReport()

bool GeoSelect::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "Config:  " << endl;
  m_msgs << "  Polys: " << m_map_polys.size() << endl;
  m_msgs << "  Segls: " << m_map_segls.size() << endl;
  m_msgs << "  Flags: " << m_poly_flags.size() << endl;
  m_msgs << "State:   " << endl;
  m_msgs << "  active_poly: " << m_active_poly << endl;
  m_msgs << "  active_segl: " << m_active_segl << endl;
  m_msgs << endl;

  ACTable actab(2);
  actab << "Poly | Spec";
  actab.addHeaderLines();

  map<string, XYPolygon>::iterator p;
  for(p=m_map_polys.begin(); p!=m_map_polys.end(); p++) {
    string label = p->first;
    string spec = p->second.get_spec();
    actab << label << spec;
  }

  m_msgs << actab.getFormattedString();

  return(true);
}




