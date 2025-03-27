/************************************************************/
/*    NAME: Mikala Molina                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: VisitPoint.cpp                                     */
/*    DATE: June 14 2023                                   */
/************************************************************/

#include <vector>
#include <iterator>
#include "MBUtils.h"
#include "VisitPoint.h"
#include "XYPoint.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

VisitPoint::VisitPoint(std::string id)
{
  m_x = 0;
  m_y = 0;
  m_state = "unvisited";
  m_id = "none";
  m_vehicle = "none";
}

//---------------------------------------------------------
// Procedure: initXY()

void VisitPoint::initXY(double x, double y)
{
  m_x = x;
  m_y = y;
}

void VisitPoint::initXY(XYPoint point)
{
  m_x = point.get_vx();
  m_y = point.get_vy();
}
//---------------------------------------------------------
// Procedure: setX()
void VisitPoint::setX(double v)
{
  m_x = v;
}
//---------------------------------------------------------
// Procedure: setY()
void VisitPoint::setY(double v)
{
  m_y = v;
}
//---------------------------------------------------------
// Procedure: setState()

bool VisitPoint::setState(string str)
{
  str = tolower(str);
  if((str != "visited") && (str != "unvisited"))
    return(false);

  m_state = str;
    return(true);
}
//---------------------------------------------------------
// Procedure: setID()
void VisitPoint::setID(std::string id)
{
  m_id = id;
}
//---------------------------------------------------------
// Procedure: setVehicle()
void VisitPoint::setVehicle(std::string s)
{
  m_vehicle = tolower(s);
}
//---------------------------------------------------------
// Procedure: getX()
double VisitPoint::getX()
{
  return(m_x);
}
//---------------------------------------------------------
// Procedure: getY()
double VisitPoint::getY()
{
  return(m_y);
}
//---------------------------------------------------------
// Procedure: getState()
string VisitPoint::getState()
{
  return(m_state);
}
//---------------------------------------------------------
// Procedure: getID()
string VisitPoint::getID()
{
  return(m_id);
}
//---------------------------------------------------------
// Procedure: getVehicle()
string VisitPoint::getVehicle()
{
  return(m_vehicle);
}
//---------------------------------------------------------
// Procedure: getSpec()
// id=1, vehicle=alpha, state=unvisited, x=42.0, y=42.0

string VisitPoint::getSpec()
{
  string spec = "id=" + m_id; 
  spec += "vehicle=" + m_vehicle;
  spec += "state=" + m_state;
  spec += "x=" + doubleToStringX(m_x,2);
  spec += "y=" + doubleToStringX(m_y,2);

  return(spec);
}

//---------------------------------------------------------
// Procedure: stringToVisitPoint()
VisitPoint stringToVisitPoint(std::string str)
{
    VisitPoint null_visit_point; 
    VisitPoint visit_point; 

    vector<string> svector = parseString(str, ',');
    for(unsigned int i=0; i<svector.size(); i++) {
        string param = tolower(biteStringX(svector[i], '='));
        string value = svector[i];
        double dval  = atof(value.c_str());

        if(param == "id")
            visit_point.setID(value);
        else if(param == "vehicle")
            visit_point.setVehicle(value);
        else if(param == "state")
            visit_point.setState(value);
        else if(param == "x")
            visit_point.setX(dval);
        else if(param == "y")
            visit_point.setY(dval);
        else
            return(null_visit_point);
    }

    return(visit_point);

}

//---------------------------------------------------------
// Procedure: getSwimmerAlert()
//formats a string  x=12, y=19, id=2 
//for compatability with 2.680 Rescue Labs

std::string VisitPoint::getSwimmerAlert()
{
    string spec = "x="+ doubleToString(m_x);
    spec += ", y="+ doubleToString(m_y); 
    spec += ", id="+m_id; 

    return(spec); 
}
//--------------------------------------------------------
// Procedure: getFoundSwimmer()
//formats a string id=2, finder=vname
//for compatability with 2.680 rescue labs

std::string VisitPoint::getFoundSwimmer()
{
    string spec = "id="+ m_id + ", finder="+ tolower(m_vehicle); 
    return(spec); 
}

//---------------------------------------------------------
// Procedure: getXYPt()
XYPoint VisitPoint::getXYPt()
{
    XYPoint point(m_x, m_y); 
    return(point); 
}