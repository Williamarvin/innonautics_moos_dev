/************************************************************/
/*    NAME: Mikala Molina                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: VisitPoint.h                                       */
/*    DATE: June 14 2022                                   */
/************************************************************/

#ifndef VisitPoint_HEADER
#define VisitPoint_HEADER

#include <string>
#include <list>
#include "XYPoint.h"

class VisitPoint
{
   public:
  VisitPoint(std::string id="");
  ~VisitPoint() {}; 

  void   initXY(double, double);
  void   initXY(XYPoint);

 public: // Setters
  void   setX(double v);
  void   setY(double v);
  bool   setState(std::string s);
  void   setID(std::string id);
  void   setVehicle(std::string s);
 
  
 public: // Getters
  double getX();
  double getY();
  std::string  getState();
  std::string  getID();
  std::string  getVehicle();
  std::string getSpec();
  std::string getSwimmerAlert(); 
  std::string getFoundSwimmer(); 
  XYPoint getXYPt();

 private:  
  double       m_x;
  double       m_y;
  std::string  m_state;        // visited or unvisted
  std::string  m_id;           //unique identifier
  std::string  m_vehicle;      //vehicle assigned to the point
  

};

VisitPoint stringToVisitPoint(std::string);


#endif  
