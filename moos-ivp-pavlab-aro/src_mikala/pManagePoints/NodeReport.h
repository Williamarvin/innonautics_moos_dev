/************************************************************/
/*    NAME: Mikala Molina                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: NodeReport.h                                       */
/*    DATE: June 26 2022                                   */
/************************************************************/

#ifndef NodeReport_HEADER
#define NodeReport_HEADER

#include <string>
#include <list>
#include "XYPoint.h"

class NodeReport
{
   public:
  NodeReport(std::string name="");
  ~NodeReport() {}; 
  public: // Setters
  void   setX(double v);
  void   setY(double v);
  void   setName(std::string s);
  void   setType(std::string s);
  void   setSpeed(double v);
  void   setHeading(double v);
  void   setLat(double v);
  void   setLon(double v);
  void   setDepth(double v);
  void   setLength(double v);
  void   setYaw(double v);
  void   setMode(std::string s);

 public: // Getters
  double getX();
  double getY();
  std::string  getName();
  std::string  getType();
  std::string  getSpec();
  std::string  getMode();
  double       getSpeed();
  double       getHeading();
  double       getLat();
  double       getLon();
  double       getDepth();
  double       getLength();
  double       getYaw(); 
  XYPoint      getXYPoint();


 private:  
  double       m_x;
  double       m_y;
  std::string  m_name;      
  std::string  m_type;
  std::string  m_mode;
  double       m_speed;
  double       m_heading;
  double       m_lat;
  double       m_lon;
  double       m_depth;
  double       m_length;
  double       m_yaw; 
  

};

NodeReport stringToNodeReport(std::string);

#endif  
