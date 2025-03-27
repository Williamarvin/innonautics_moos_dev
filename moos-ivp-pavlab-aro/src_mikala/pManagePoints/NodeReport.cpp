/************************************************************/
/*    NAME: Mikala Molina                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: NodeReport.cpp                                     */
/*    DATE: June 26 2023                                   */
/************************************************************/

#include <vector>
#include <iterator>
#include "MBUtils.h"
#include "NodeReport.h"
#include "XYPoint.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

NodeReport::NodeReport(std::string name)
{
  m_x = 0;
  m_y = 0;
  m_name = "none";
  m_type = "none";
  m_lat = 0;
  m_lon = 0;
  m_speed = 0;
  m_heading = 0;
  m_yaw = 0;
  m_depth = 0;
  m_length = 0;
  m_mode = "none";

}
//---------------------------------------------------------
// Procedure: setX()
void NodeReport::setX(double v)
{
  m_x = v;
}
//---------------------------------------------------------
// Procedure: setY()
void NodeReport::setY(double v)
{
  m_y = v;
}
//---------------------------------------------------------
// Procedure: setLat()
void NodeReport::setLat(double v)
{
  m_lat = v;
}
//---------------------------------------------------------
// Procedure: setLon()
void NodeReport::setLon(double v)
{
  m_lon = v;
}
//---------------------------------------------------------
// Procedure: setSpeed()
void NodeReport::setSpeed(double v)
{
  m_speed = v;
}
//---------------------------------------------------------
// Procedure: setHeading()
void NodeReport::setHeading(double v)
{
  m_heading = v;
}
//---------------------------------------------------------
// Procedure: setYaw()
void NodeReport::setYaw(double v)
{
  m_yaw = v;
}
//---------------------------------------------------------
// Procedure: setDepth()
void NodeReport::setDepth(double v)
{
  m_depth = v;
}
//---------------------------------------------------------
// Procedure: setLength()
void NodeReport::setLength(double v)
{
  m_length = v;
}
//---------------------------------------------------------
// Procedure: setName()
void NodeReport::setName(std::string v)
{
  m_name = v;
}
//---------------------------------------------------------
// Procedure: setType()
void NodeReport::setType(std::string v)
{
  m_type = v;
}
//---------------------------------------------------------
// Procedure: setMode()
void NodeReport::setMode(std::string v)
{
  m_mode = v;
}
//---------------------------------------------------------
//Procedure: stringToNodeReport()

NodeReport stringToNodeReport(std::string str)
{
  NodeReport empty_node_report; 
  NodeReport node_report;

  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(biteStringX(svector[i], '='));
    string value = svector[i];
    string unhandled_contents = ""; 
    double dval  = atof(value.c_str());

      if(param == "NAME" || param == "name")
        node_report.setName(value);
      else if(param == "TYPE" || param == "type")
        node_report.setType(value);
      else if(param == "X" || param == "x")
        node_report.setX(dval);
      else if(param == "Y" || param == "y")
        node_report.setY(dval);
      else if(param == "LAT" || param == "lat")
        node_report.setLat(dval);
      else if(param == "LON" || param == "lon")
        node_report.setLon(dval);
      else if(param == "SPEED" || param == "speed")
        node_report.setSpeed(dval);
      else if(param == "HEADING" || param == "heading")
        node_report.setHeading(dval);
      else if(param == "YAW" || param == "yaw")
        node_report.setYaw(dval);
      else if(param == "DEPTH" || param == "depth")
        node_report.setDepth(dval);
      else if(param == "LENGTH" || param == "length")
        node_report.setLength(dval);
      else if(param == "MODE" || param == "mode")
        node_report.setMode(value);
      else
       unhandled_contents += svector[i] + ", ";
        

        
      
  }
  return(node_report);
}
//---------------------------------------------------------
// Procedure: getX()
double NodeReport::getX()
{
  return(m_x);
}
//---------------------------------------------------------
// Procedure: getY()
double NodeReport::getY()
{
  return(m_y);
}
//---------------------------------------------------------
// Procedure: getXYPoint()
XYPoint NodeReport::getXYPoint()
{
  XYPoint point(m_x, m_y);
  return(point);
}
//---------------------------------------------------------
// Procedure: getLat()
double NodeReport::getLat()
{
  return(m_lat);
}
//---------------------------------------------------------
// Procedure: getLon()
double NodeReport::getLon()
{
  return(m_lon);
}
//---------------------------------------------------------
// Procedure: getSpeed()
double NodeReport::getSpeed()
{
  return(m_speed);
}
//---------------------------------------------------------
// Procedure: getHeading()
double NodeReport::getHeading()
{
  return(m_heading);
}
//---------------------------------------------------------
// Procedure: getYaw()
double NodeReport::getYaw()
{
  return(m_yaw);
}
//---------------------------------------------------------
// Procedure: getDepth()
double NodeReport::getDepth()
{
  return(m_depth);
}
//---------------------------------------------------------
// Procedure: getLength()
double NodeReport::getLength()
{
  return(m_length);
}
//---------------------------------------------------------
// Procedure: getName()
std::string NodeReport::getName()
{
  return(m_name);
}
//---------------------------------------------------------
// Procedure: getType()
std::string NodeReport::getType()
{
  return(m_type);
}
//---------------------------------------------------------
// Procedure: getMode()
std::string NodeReport::getMode()
{
  return(m_mode);
}
//---------------------------------------------------------
