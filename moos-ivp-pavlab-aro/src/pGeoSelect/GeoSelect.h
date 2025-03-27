/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GeoSelect.h                                     */
/*    DATE: July 7th, 2022                                  */
/************************************************************/

#ifndef GEO_SELECT_HEADER
#define GEO_SELECT_HEADER

#include <map>
#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPolygon.h"
#include "XYSegList.h"
#include "VarDataPair.h"

class GeoSelect : public AppCastingMOOSApp
{
 public:
  GeoSelect();
  ~GeoSelect() {};

 protected: // Standard MOOSApp functions to overload  
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
  bool buildReport();

 protected:
  void registerVariables();
  
  bool handleConfigPolygon(std::string);
  bool handleConfigSegList(std::string);

  bool handleMailGSPolyReq(std::string);
  bool handleMailGSPolyOff(std::string);
  bool handleMailGSPolyAct(std::string s="");

 protected:
  void drawPoly(XYPolygon);
  void erasePoly(XYPolygon);

  void postFlags(const std::vector<VarDataPair>&);
  
  
 private: // Configuration variables

 private: // State variables
  std::string m_active_poly;
  std::string m_active_segl;
  
  std::map<std::string, XYPolygon> m_map_polys;
  std::map<std::string, XYSegList> m_map_segls;

  std::vector<VarDataPair> m_poly_flags;
};

#endif 
