/************************************************************/
/*    NAME: Mikala Molina                                    */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SendPoints.cpp                     */
/*    DATE: June 1 2023                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SendPoints.h" 
#include "XYFormatUtilsPoint.h"
#include "XYFormatUtilsSegl.h"
#include "GeomUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

SendPoints::SendPoints()
{
  m_need_to_send = false;
  m_min_dist = 10;
  m_all_initial_points_sent = false;
  m_minimum_points = 1;
  m_capture_radius = 10;
}

//---------------------------------------------------------
// Destructor

SendPoints::~SendPoints()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool SendPoints::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    string sval = msg.GetString();
    string comm = msg.GetCommunity();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if (key == "NODE_REPORT")
    {
      cout << "Node Report Received: " << sval << endl;
      NodeReport report = stringToNodeReport(sval);
      string vname = report.getName();
      // add the vname to m_vehicle_names if it is not already in the vector
      if (find(m_vehicle_names.begin(), m_vehicle_names.end(), vname) == m_vehicle_names.end())
      {
        m_vehicle_names.push_back(vname);
        m_vehicle_awake[vname] = true;
        m_id_counter[vname] = 0;
        m_needs_initial_points[vname] = true;
      }
      for (int i = 0; i < m_number_of_vehicles; i++)
      {
        if (tolower(vname) == m_vehicle_names[i])
        {
          m_nav_xy_points[m_vehicle_names[i]].push_back(report.getXYPoint());
        }
      }
    }
    else if (key == "NEED_POINTS")
    {
      string comm = msg.GetCommunity();
      string vname = comm;
      sval = msg.GetString();
      m_map_need_points[vname] = (sval == "true");
      Notify("NEED_POINTS_REC", vname + " " + sval);
    }
    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool SendPoints::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SendPoints::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Check if a vehicle is awake. If it is, generate m_initial points number of initial points and create an XYSeglist of those points
  for (int i = 0; i < m_number_of_vehicles; i++)
  {
    string vname = m_vehicle_names[i];
    if (m_vehicle_awake[vname] == true && m_needs_initial_points[vname] == true)
    {
      for (int j = 0; j < m_initial_points; j++)
      {
        XYPoint point = generateNewPoint();
        VisitPoint vpoint;
        vpoint.initXY(point);
        vpoint.setID(intToString(m_id_counter[vname]));
        vpoint.setState("unvisited");
        m_visit_points[vname].push_back(vpoint);
        m_id_counter[vname]++;
      }
      for (unsigned int j = 0; j < m_visit_points[vname].size(); j++)
      {
        m_visit_point_seglists[vname].add_vertex(m_visit_points[vname][j].getX(), m_visit_points[vname][j].getY());
      }
      if (m_visit_point_seglists[vname].get_spec() != "")
      {
        Notify("SURVEY_UPDATE_" + toupper(vname), "points=" + m_visit_point_seglists[vname].get_spec());
        m_needs_initial_points[vname] = false;
        debug1 = "points sent to " + vname;
      }
    }
  }
  // if all vehicles have received their initial points, set m_all_initial_points_sent to true
  bool all_initial_points_sent = true;
  for (int i = 0; i < m_number_of_vehicles; i++)
  {
    string vname = m_vehicle_names[i];
    if (m_needs_initial_points[vname] == true)
    {
      all_initial_points_sent = false;
      break;
    }
  }
  if (all_initial_points_sent == true)
  {
    m_all_initial_points_sent = true;
  }

  // once all inital points are sent, start checking for visited points
  if (m_all_initial_points_sent == true)
  {
    for (int i = 0; i < m_number_of_vehicles; i++)
    {
      string vname = m_vehicle_names[i];
      for (unsigned int j = 0; j < m_visit_points[vname].size(); j++)
      {
        bool checked = checkVisited(m_visit_points[vname][j], vname);
        if (checked == true)
        {
          m_visit_points[vname][j].setState("visited");
          XYSegList unvisited_seglist;
          for (unsigned int k = 0; k < m_visit_points[vname].size(); k++)
          {
            if (m_visit_points[vname][k].getState() == "unvisited")
            {
              unvisited_seglist.add_vertex(m_visit_points[vname][k].getX(), m_visit_points[vname][k].getY());
              m_map_unvisited_points[vname].push_back(m_visit_points[vname][k]);
            }
          }
          if (unvisited_seglist.get_spec() != "")
          {
            Notify("SURVEY_UPDATE_" + toupper(vname), "points=" + unvisited_seglist.get_spec());
          }
        }
      }
    }
  }

  // count the number of unvisited points for each vehicle
  for (int i = 0; i < m_number_of_vehicles; i++)
  {
    string vname = m_vehicle_names[i];
    m_unvisited_count[vname] = countUnvisited(vname);
  }

  // count the number of visited points for each vehicle
  for (int i = 0; i < m_number_of_vehicles; i++)
  {
    string vname = m_vehicle_names[i];
    int count = 0;
    for (unsigned int j = 0; j < m_visit_points[vname].size(); j++)
    {
      if (m_visit_points[vname][j].getState() == "visited")
      {
        count++;
      }
      m_visited_count[vname] = count;
    }
  }

  for (int i = 0; i < m_number_of_vehicles; i++)
  {
    string vname = m_vehicle_names[i];
    if (m_map_need_points[vname] == true)
    {
      for (int j = 0; j < m_minimum_points; j++)
      {
        XYPoint new_point = generateNewPoint();
        VisitPoint vpoint;
        vpoint.initXY(new_point);
        vpoint.setID(intToString(m_id_counter[vname]++));
        vpoint.setState("unvisited"); // Explicitly set state to unvisited
        m_visit_points[vname].push_back(vpoint);
        m_map_unvisited_points[vname].push_back(vpoint);

        // Add to seglist for notification
        XYSegList unvisited_seglist;
        for (unsigned int k = 0; k < m_map_unvisited_points[vname].size(); k++)
        {
          unvisited_seglist.add_vertex(m_map_unvisited_points[vname][k].getX(), m_map_unvisited_points[vname][k].getY());
        }
        if (unvisited_seglist.get_spec() != "")
        {
          //Notify("SURVEY_UPDATE_" + toupper(vname), "points=" + unvisited_seglist.get_spec());
          Notify("STATION_KEEP_" + toupper(vname), "false");
          Notify("SURVEY_" + toupper(vname), "true");
          Notify("DEPLOY_" + toupper(vname), "true");
        }
      }
      m_map_need_points[vname] = false;
    }
  }

  AppCastingMOOSApp::PostReport();
  return (true);

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SendPoints::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;

    if (param == "initial_points")
    {
      m_initial_points = stoi(value);
      handled = true;
      cout << "Initial Points from parameters: " << m_initial_points << endl;
    }
    else if (param == "minimum_points")
    {
      m_minimum_points = stoi(value);
      handled = true;
      cout << "Minimum Points from parameters: " << m_minimum_points << endl;
    }
    else if (param == "capture_radius")
    {
      m_capture_radius = stoi(value);
      handled = true;
      cout << "Capture Radius from parameters: " << m_capture_radius << endl;
    }
    else if (param == "min_distance")
    {
      m_min_dist = stoi(value);
      handled = true;
      cout << "minimum distance enforced between points (default 10): " << m_min_dist << endl;
    }
    else if (param == "region")
    {
      m_region = value;
      if (m_region == "mit")
      {
        m_region_poly.add_vertex(60, 10);
        m_region_poly.add_vertex(-30.3602, -32.8374);
        m_region_poly.add_vertex(-4.6578, -87.0535);
        m_region_poly.add_vertex(85.7024, -44.2161);
        m_region_fld.addPolygon(m_region_poly);
        Notify("VIEW_POLYGON", m_region_poly.get_spec());
      }
      handled = true;
    }
    else if (param == "amount")
    {
      m_number_of_vehicles = stoi(value);
      handled = true;
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void SendPoints::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register("NEED_POINTS", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool SendPoints::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: SendPoints.cpp " << endl;
  m_msgs << "============================================" << endl;
  // generate a table of the vehicle names, if they are awake, if they need initial points, and how many points they have received
  ACTable actab(6);
  actab << "Vehicle"
        << "Awake"
        << "Needs Initial"
        << "Points Received"
        << "Unvisited Points"
        << "Visited Points";
  for (int i = 0; i < m_number_of_vehicles; i++)
  {
    string vname = m_vehicle_names[i];
    actab << vname << boolToString(m_vehicle_awake[vname]) << boolToString(m_needs_initial_points[vname]) << intToString(m_id_counter[vname]) << intToString(m_unvisited_count[vname]) << intToString(m_visited_count[vname]);
  }
  m_msgs << actab.getFormattedString() << endl;
  m_msgs << "============================================" << endl;
  m_msgs << debug3 << endl;
  m_msgs << "============================================" << endl;

  // generate a table to display m_map_station_keep
  ACTable actab2(2);
  actab2 << "Vehicle"
         << "Need Points";
  for (int i = 0; i < m_number_of_vehicles; i++)
  {
    string vname = m_vehicle_names[i];
    actab2 << vname << boolToString(m_map_need_points[vname]);
  }
  m_msgs << actab2.getFormattedString() << endl;

  m_msgs << "============================================" << endl;
  m_msgs << "Capture Radius: " << m_capture_radius << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Minimum points per vehicle: " << m_minimum_points << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "All Intial points received? " << boolToString(m_all_initial_points_sent) << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Minimum Distance" << m_min_dist << endl;
  m_msgs << "============================================" << endl;
  m_msgs << debug1 << endl;
  m_msgs << "Message here if a vehicle has visited all points: " << debug2 << endl;

  return (true);
}
//-----------------------------------------------------------------
// Procedure: generateNewPoint()
// generates a new point within the region and ensures the point is atleast m_minimum_distance away from all unvisted points
XYPoint SendPoints::generateNewPoint()
{
  XYPoint point = m_region_fld.generatePoint();
  for (int i = 0; i < m_number_of_vehicles; i++)
  {
    string vname = m_vehicle_names[i];
    for (unsigned int j = 0; j < m_map_unvisited_points[vname].size(); j++)
    {
      double dist = distPointToPoint(point.get_vx(), point.get_vy(), m_map_unvisited_points[vname][j].getX(), m_map_unvisited_points[vname][j].getY());
      if (dist < m_min_dist)
      {
        point = m_region_fld.generatePoint();
      }
    }
  }

  return point;

}
//-----------------------------------------------------------------
// Procedure: checkVisited(XYPoint point)
// checks if a point is within m_capture_radius of a vehicles m_nav_xy_points vector
bool SendPoints::checkVisited(VisitPoint point, string vname)
{
  bool visited = false;
  for (unsigned int i = 0; i < m_nav_xy_points[vname].size(); i++)
  {
    double dist = distPointToPoint(point.getX(), point.getY(), m_nav_xy_points[vname][i].get_vx(), m_nav_xy_points[vname][i].get_vy());
    if (dist < m_capture_radius)
    {
      visited = true;
      point.setState("visited");
      // break;
    }
  }
  return visited;
}
//-----------------------------------------------------------------
// Procedure: countUnvisited(string vname)
// counts the number of unvisited points for a vehicle
int SendPoints::countUnvisited(string vname)
{
  int count = 0;
  for (unsigned int i = 0; i < m_visit_points[vname].size(); i++)
  {
    if (m_visit_points[vname][i].getState() == "unvisited")
    {
      count++;
    }
  }
  return count;
}
