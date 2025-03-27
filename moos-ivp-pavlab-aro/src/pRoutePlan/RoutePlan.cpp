/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RoutePlan.cpp                                        */
/*    DATE: December 29th, 1963                             */
/***********************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "RoutePlan.h"
#include "XYFormatUtilsConvexGrid.h"
#include <map>
#include <set>
#include <vector>
#include "EsriBathyGridUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor

RoutePlan::RoutePlan()
{
  m_path_color_map["gus"] = "yellow";
  m_path_color_map["ida"] = "white";
  m_path_color_map["jing"] = "red";
  m_path_color_map["kirk"] = "macpurple";
  
}

//---------------------------------------------------------
// Destructor

RoutePlan::~RoutePlan()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool RoutePlan::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);
  
  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();


    if (key == "VIEW_GRID_CONS_LOCAL") {
      string label = getGridLabel(sval);
      bool cond = (label == "cons"); // label should always be cons
      if (cond) {
	XYConvexGrid temp_grid = string2ConvexGrid(sval);
	m_grid.setXYConvexGrid(temp_grid);
	cout << "GRID RECEIVED" << endl;
	m_full_grid_rcvd++;
	
	m_new_grid_received = true;
	m_time_last_grid_was_received = MOOSTime();
	m_proposal_iterations = 0;
	m_won_a_proposal = false;
	
	if (!m_first_grid_received) {
	  // This is the first grid we have recieved.
	  // use it to preload the A* graph
	
	  // get A* params
	  m_vertices = m_grid.getVertices();
	  m_edges = m_grid.getEdges();

	  // add the other start and goal nodes if needed
	  if (m_use_all_top_cells_as_start) {
	    std::set<unsigned int> m_top_start = m_grid.getCellsTop();
	    m_start_ids.insert(m_top_start.begin(), m_top_start.end());
	  }

	  if (m_use_all_bottom_cells_as_goal) {
	    std::set<unsigned int> m_bottom_goal = m_grid.getCellsBottom();
	    m_end_ids.insert(m_bottom_goal.begin(), m_bottom_goal.end());
	  }

	  bool sort_by_asc_cell_id = false;
	  bool ok = m_astar.preloadGraph(m_vertices, m_edges, m_start_ids, m_end_ids, sort_by_asc_cell_id);
	  if (not ok)
	    reportRunWarning("Error: was not able to preload graph in A*");


	  m_first_grid_received = true;
	} // end of if first grid not recieved. 
	
      } // end of if consensus grid

    } else if (key == "VIEW_GRID_CONS_LOCAL_DELTA") {
      if (!m_first_grid_received)
	reportRunWarning("Mail Error: Recieved a grid delta message before receiving the full grid.");
      
      bool ok1 = m_grid.processGridDelta(sval);
      if (not ok1)
	reportRunWarning("Mail Error: Was not able to process grid delta" + sval);

      m_full_grid_deltas_rcvd++;
      m_new_grid_received = true;
      m_time_last_grid_was_received = MOOSTime();
      m_proposal_iterations = 0;
      m_won_a_proposal = false;
      
    } else if (key == "SURVEYING") {
      if (m_surveying == false)
	m_surveying = (sval == "true");

      if (m_start_time == -1)
	m_start_time = MOOSTime(); // init start of survey;

    } else if (key=="EXPLORING") {
      
      if (m_start_time == -1)
	m_start_time = MOOSTime(); // init start of survey;
      
      // update waypoints if needed
      updateStartOrEndPaths();

    } else if (key == m_proposal_var_name) {
      bool ok = handlePathProposal(sval, MOOSTime() );
      if (not ok)
	reportRunWarning("A path proposal was not correctly formated: " + sval);
      
    } else if (key == "NAV_X") {
      m_nav_x = dval;
      
    } else if (key == "NAV_Y") {
      m_nav_y = dval;

    } else if (key == "CYCLE_INDEX") {
      // just completed a cycle of points,
      // if we have a viable path, then update
      // the waypoints to be the other direction
      // along the path.
      // This is stored as the "reverse path"
      if (m_found_path_that_may_be_viable) {
	if (m_rev_path.size() > 1) {
	this->updateWaypointPath(m_rev_path);

	std::vector<std::size_t> temp_path = m_fwd_path;
	m_fwd_path = m_rev_path;
	m_rev_path = temp_path;
	} else {
	  // there is no reverse path if the vehicle
	  // started at one end. The size will
	  // just be equal to one, only the starting cell
	  // In this case, just reverse
	  // the points and update
	  std::reverse(m_fwd_path.begin(), m_fwd_path.end() );
	  this->updateWaypointPath(m_fwd_path);
	  // no need to update the rev path, it is empty
	}
	
      }
      
    } else if (key == "ADD_END") {

      double x = stod(biteStringX(sval,','));
      double y = stod(sval);
      int id;

      if (m_grid.getCellID(x, y, id))
	m_end_ids.insert(id);
      
      plotEndIds();
      // update waypoints if needed
      updateStartOrEndPaths();
      
    } else if (key == "ADD_START") {

      double x = stod(biteStringX(sval,','));
      double y = stod(sval);
      int id;

      if (m_grid.getCellID(x, y, id))
	m_start_ids.insert(id);

      plotStartIds();
      // update waypoints if needed
      updateStartOrEndPaths();

    } else if (key == "CLEAR_END") {

      m_end_ids = {};
      plotEndIds();
      // update waypoints if needed
      updateStartOrEndPaths();
      
    } else if (key == "CLEAR_START") {

      m_start_ids = {};
      plotStartIds();
      // update waypoints if needed
      updateStartOrEndPaths();
      
    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  
  return(true);
}

//--------------------------------------------------------
// Procedure: OnConnectToServer

bool RoutePlan::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool RoutePlan::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  // Bail right away if not surveying
  // or no new grid is recieved.
  // m_new_grid_recieved is the flag that indicates a new grid
  // was recieved (obviously) and we are not done processing it
  // Also bail if we have already found the final path
  
  // Also bail if we have not waited until the consensus is finished.
  double time_elapsed_since_last_grid = MOOSTime() - m_time_last_grid_was_received;
  bool cond1 = (m_new_grid_received and (time_elapsed_since_last_grid < (m_consensus_wait_time + 0.5)) );
  
  if (!m_surveying or !m_new_grid_received or cond1 or m_final_path_found) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }
 
  // Search for a final path
  // first set the threshold, then get the final obstacles
  m_var_threshold = calcDynamicThreshold();
  m_final_obstacles = m_grid.getFinalObstacles(m_depth_threshold, m_var_threshold);

  string obs_out = "";
  for (set<size_t>::const_iterator it = m_final_obstacles.cbegin(); it != m_final_obstacles.cend();) {
    obs_out += to_string(*it);
    ++it;
    if (it != m_final_obstacles.cend())
      obs_out += " ";
  }

  Notify("FINAL_OBSTACLES", obs_out);

  m_final_path = m_astar.searchPathFast(m_final_obstacles);

  bool found_path = (m_final_path.size() > 0) and (not m_sent_final_path_found); // found a path
  bool timed_out = m_timeout and ((MOOSTime() - m_start_time) > m_timeout_time); // timed out

  // There are two end conditions, we found a path or we timed out. 
  if (found_path) {

    string route_str = "";
    for (vector<size_t>::const_iterator it = m_final_path.cbegin(); it != m_final_path.cend();) {
      route_str += to_string(*it);
      ++it;
      if (it != m_final_path.cend())
	route_str += " ";
    }
    
    m_final_path_found = true;
    m_time_to_first_path = MOOSTime();
    Notify("ROUTE_FOUND", "1");
    Notify("FINAL_ROUTE", route_str);
    Notify("RETURN", "true");
    Notify("TIME_TO_ROUTE", to_string(m_time_to_first_path - m_start_time));
    m_sent_final_path_found = true;
    this->plotPath(m_final_path);

  } else if (timed_out) {

    Notify("ROUTE_FOUND", "1");
    Notify("FINAL_ROUTE", "timeout");
    Notify("RETURN", "true");
    Notify("TIME_TO_ROUTE", to_string(m_time_to_first_path - m_start_time));
    
  }

  // bail if basic lawnmower or just found path
  if (!m_explore_possible_paths or m_final_path_found) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }
  
  // Now determine what to do.
  // There are three conditions to consider:
  bool iterations_less_than_max = m_proposal_iterations < m_max_proposal_iterations;
  
  // the time since the last proposal will be large when a new grid comes in 
  // thus we can use it as a criteria here.
  bool enough_time_elapsed_since_last_proposal = (MOOSTime() - m_last_proposal_time) > m_proposal_wait_time;
  // The third condition is whether we have won a proposal.
  if (iterations_less_than_max and enough_time_elapsed_since_last_proposal and (not m_won_a_proposal)) {

    // We are ready to check all the valid proposals to see if we won.
    m_won_a_proposal = checkProposalsForWin();

    if (m_won_a_proposal) {

      // update waypoints
      // Check if this is a new set of waypoints, and update
      // only if it is new.
      if (m_p != m_proposed_paths.at(m_vname)) {
	// We want to start at the vertex closest to us, and
	// travel in the direction of most variance (least confidence)
	m_fwd_path = this->findBestPathDirection();
	
	this->updateWaypointPath(m_fwd_path);
	m_p = m_proposed_paths.at(m_vname);
      } 

      // plot path
      this->plotPath( m_proposed_paths.at(m_vname) );
      
      Notify("PATH_FOUND", "true");
      m_found_path_that_may_be_viable = true;
      m_first_proposal_won = true;
      
      // reset flags
      m_proposal_iterations = 0;
      m_new_grid_received = false;
      
    } else {
      // we didn't win, or this is a new grid so we need
      // to generate a new proposed path.
      
      // First get the actual obstacles found in the grid
      m_obstacles.clear();
      m_var_threshold = calcDynamicThreshold();
      m_obstacles = m_grid.getObstacles(m_depth_threshold, m_var_threshold);

      // Add border cells to obstacles (border effects)
      size_t min_start = *m_start_ids.begin();
      size_t max_start = *m_start_ids.rbegin();
      size_t min_end = *m_end_ids.begin();
      size_t max_end = *m_end_ids.rbegin();
      m_obstacles.insert(min_start);
      m_obstacles.insert(max_start);
      m_obstacles.insert(min_end);
      m_obstacles.insert(max_end);

      size_t min_start2 = *(next(m_start_ids.begin(),1));
      size_t max_start2 = *(next(m_start_ids.rbegin(),1));
      size_t min_end2 = *(next(m_end_ids.begin(),1));
      size_t max_end2 = *(next(m_end_ids.rbegin(),1));
      m_obstacles.insert(min_start2);
      m_obstacles.insert(max_start2);
      m_obstacles.insert(min_end2);
      m_obstacles.insert(max_end2);
      
      string obs_spec = m_grid.getObstacleGridSpec(m_depth_threshold, m_var_threshold);
      Notify("VIEW_GRID_OBS_LOCAL", obs_spec);
      
      // Set all the vertices of the valid paths to be obstacles
      // It is ok if there are any reduntant paths, because at 
      // least someone would have taken that path, and the 
      // set container ensures unique elements.
      std::set<size_t> path_obstacles = getVerticesFromValidPaths();
      
      m_obstacles.insert( path_obstacles.begin(), path_obstacles.end() );

      // Run the path planner
      std::vector<std::size_t> path;
      path = m_astar.searchPathFast(m_obstacles);
      if (path.size() < 1) {
	// no path was found.
	// clean up an post flags.
	Notify("PATH_FOUND", "false");

	m_found_path_that_may_be_viable = false;
	// clear the path, by basically plotting an empty path
	this->plotPath(path);
	// We are done with this attempt. 
	m_new_grid_received = false;
	m_proposal_iterations = 0;
	m_no_path_counter++;
	AppCastingMOOSApp::PostReport();
	return(true);
      } else {
	
	// A new path was found.
	// Only adopt this path if either of the three conditions are met
	// 1.  The old path that was previously awarded to
	//     us is no longer viable.
	bool old_path_is_viable = isViablePath(m_p);
	// 2.  This path is shorter, and thus more optimal
	bool new_path_is_shorter = isPathShorter(path, m_p);
	// 3.  We have completed more than the max number of passes
	bool exceeded_limit = m_path_maintained_cnt >= m_path_maintained_max;
	// implemented this way for readability 
	if ( (not old_path_is_viable ) or new_path_is_shorter or exceeded_limit )  {
	  path = path;
	  m_path_maintained_cnt = 0;
	} else {
	  path = m_p;
	  m_path_maintained_cnt ++;
	}
      }
      
      // Calculate the cost to the closest index on the path
      std::vector<double> current_position = {m_nav_x, m_nav_y};
      std::vector<std::vector<double> > path_vertices;
      for (unsigned int j=0; j<path.size(); j++) {
	path_vertices.push_back(m_vertices.at(path[j]));
      }
      std::size_t index_of_closest_vertex; // not used here;
      double cost_to_path = calcCost(current_position, path_vertices, index_of_closest_vertex);

      cout << "Sending new proposal" << endl;
      
      // Send out this proposal
      std::string proposal_spec = getPathProposalSpec(path, cost_to_path);
      this->sendOutOwnProposal(proposal_spec);

      // Save this as own proposal
      m_proposed_paths[m_vname] = path;
      m_proposed_costs[m_vname] = cost_to_path;
      m_proposal_arrival_time[m_vname] = MOOSTime();

      // Update the flags
      m_last_proposal_time = MOOSTime();
      m_proposal_iterations++;
    }
    
  }


  // Clean up if needed
  if (not iterations_less_than_max) {
    // We are done with this attempt. 
    m_new_grid_received = false;
    m_proposal_iterations = 0;
    
  }
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool RoutePlan::OnStartUp()
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
    if (param == "depth_threshold") {
      handled = isNumber(value);
      m_depth_threshold = stod(value);
      
    } else if(param == "variance_threshold") {
      handled = isNumber(value);
      m_var_threshold = stod(value);

    } else if(param == "variance_pct_threshold") {
      handled = isNumber(value);
      m_var_pct_thresh = stod(value);
      
    } else if(param == "proposal_var_name") {
      handled = true;
      m_proposal_var_name = toupper(value);
      
    } else if(param == "consensus_wait_time") {
      handled = isNumber(value);
      m_consensus_wait_time = stod(value);

    } else if(param == "proposal_wait_time") {
      handled = isNumber(value);
      m_proposal_wait_time = stod(value);
      
    } else if(param == "max_proposal_iterations") {
      handled = isNumber(value);
      m_max_proposal_iterations = stoul(value);

    } else if(param == "max_maintain_path") {
      handled = isNumber(value);
      m_path_maintained_max = stoi(value);

    } else if(param == "explore_possible_paths") {
      handled = true;
      m_explore_possible_paths = (stoi(value)==1);

    } else if(param == "number_of_vehicles") {
      handled = true;
      m_number_of_vehicles = stoi(value);

    } else if(param == "vehicle_n") {
      handled = true;
      m_vehicle_n = stoi(value);

    } else if(param == "use_all_top_cells_as_start") {
      if ( isBoolean(value) ) {
	handled = true;
	m_use_all_top_cells_as_start = (value == "true");
      }

    } else if(param == "use_all_bottom_cells_as_goal") {
      if ( isBoolean(value) ) {
	handled = true;
	m_use_all_bottom_cells_as_goal = (value == "true");
      }

    } else if(param == "start_cells") {
      bool all_ok1 = true;
      vector<string> msgs = parseString(value, ',');
      for(unsigned int i=0; i<msgs.size(); i++) {
	string msg = stripBlankEnds(msgs[i]);
	if (isNumber(msg)){
	  m_start_ids.insert( stol(msg));
	} else {
	  all_ok1 = false;
	}
      }
      handled = all_ok1;
      
    } else if(param == "goal_cells") {
      bool all_ok2 = true;
      vector<string> msgs = parseString(value, ',');
      for(unsigned int i=0; i<msgs.size(); i++) {
	string msg = stripBlankEnds(msgs[i]);
	if (isNumber(msg)){
	  m_end_ids.insert( stol(msg));
	} else {
	  all_ok2 = false;
	}
      }
      handled = all_ok2;

    } else if(param == "timeout") {
      m_timeout = (stoi(value)==1);
      handled = true;
    }
    
    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  bool latOK = m_MissionReader.GetValue("LatOrigin", m_lat_origin);
  if(!latOK) {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return(false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", m_lon_origin);
  if(!lonOK){
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return(false);
  }

  bool nameOK = m_MissionReader.GetValue("Community", m_vname);
  if(!nameOK) {
    reportConfigWarning("Community name missing in MOOS file.");
    return(false);
  }

  XYConvexGrid temp_grid;
  m_grid = EsriBathyGrid(temp_grid,m_lat_origin,m_lon_origin,m_no_data_value);
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void RoutePlan::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VIEW_GRID_CONS_LOCAL", 0);
  Register("VIEW_GRID_CONS_LOCAL_DELTA", 0);
  Register("SURVEYING", 0);
  Register("EXPLORING", 0);
  Register(m_proposal_var_name, 0);
  Register("NAV_X",0);
  Register("NAV_Y",0);
  Register("CYCLE_INDEX",0);
  Register("ADD_START",0);
  Register("ADD_END",0);
  Register("CLEAR_START",0);
  Register("CLEAR_END",0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool RoutePlan::buildReport() 
{
  m_msgs << "Surveying = " << boolToString(m_surveying) << endl;
  m_msgs << "Survey time: " << (MOOSTime() - m_start_time) << endl;
  m_msgs << "Number of full grids received = " << m_full_grid_rcvd << endl;
  m_msgs << "Number of grid deltas recieved = " << m_full_grid_deltas_rcvd << endl;
  m_msgs << "New grid received = " << boolToString(m_new_grid_received) << endl;

  m_msgs << "--Final Path Search Status---------------------" << endl;
  double time_elapsed_since_last_grid =MOOSTime() - m_time_last_grid_was_received;
  bool cond1 = (m_new_grid_received and (time_elapsed_since_last_grid < (m_consensus_wait_time + 2.0)) );
  if (!m_surveying or !m_new_grid_received or cond1 or m_final_path_found) {
    m_msgs << "Searching for a final path" << endl;
  } else {
    m_msgs << "Not Searching for final path" << endl;
  }
  m_msgs << "Depth threshold = " << m_depth_threshold << ", Dynamic variance thresh = " << m_var_threshold << endl;
  m_msgs << "Estimated number of obstacles in grid = " << m_final_obstacles.size() << endl;
  m_msgs << "Number of start cells = " << m_start_ids.size() << " Number of end cells = " << m_end_ids.size() << endl;
  m_msgs << "Final path found = " << boolToString(m_final_path_found) << endl;
  m_msgs << "Time to find final path: " << (m_time_to_first_path - m_start_time) << endl;

  m_msgs << "Final path = {";

  for (vector<size_t>::const_iterator it = m_final_path.cbegin(); it != m_final_path.cend();) {
    m_msgs << *it;
    ++it;
    if (it != m_final_path.cend())
      m_msgs << ",";
  }

  m_msgs << "}" << endl;
  
 
  m_msgs << "----Proposal Awarding ------------------------------" << endl;
  m_msgs << "Number of neighbors that submitted proposals = " << m_proposed_paths.size() << endl;
  m_msgs << "Number of proposal iterations = " << m_proposal_iterations << endl;
  m_msgs << "Time since last proposal = " << MOOSTime() - m_last_proposal_time << endl;
  m_msgs << "Won a proposal = " << m_won_a_proposal << endl;
  m_msgs << "Number of maintained paths (hysteresis) = " << m_path_maintained_cnt << endl;
  m_msgs << "Number of times A* found no paths = " << m_no_path_counter << endl;
  m_msgs << "Number of times proposal timed out = " << m_proposal_timeout_cnt << endl; 
  m_msgs << "------Current Proposals -------------------" << endl;

  std::map<std::string,std::vector<std::size_t> >::iterator it;
  for (it = m_proposed_paths.begin(); it != m_proposed_paths.end(); it++) {
    string vname = it->first;
    vector<size_t> path = it->second;
    m_msgs << "[Vehicle=" << vname << ": Path =";
    for (vector<size_t>::const_iterator it2 = path.cbegin(); it2 != path.cend();) {
      m_msgs << *it2;
      ++it2;
      if (it2 != path.cend())
	m_msgs << ",";
    }
    m_msgs << ": Cost = "  << m_proposed_costs.at(vname);
    m_msgs << ": Time Exp = " << MOOSTime() - m_proposal_arrival_time.at(vname) << "]" << endl;
  }
  
  m_msgs << "------Proposal Won: -------------------" << endl;
  m_msgs << "Fwd Path: ";
  for (vector<size_t>::const_iterator it2 = m_fwd_path.cbegin(); it2 != m_fwd_path.cend();) {
    m_msgs << *it2;
    ++it2;
    if (it2 != m_fwd_path.cend())
      m_msgs << ",";
  }
  m_msgs << endl;

  m_msgs << "Rev Path: ";
  for (vector<size_t>::const_iterator it2 = m_rev_path.cbegin(); it2 != m_rev_path.cend();) {
    m_msgs << *it2;
    ++it2;
    if (it2 != m_rev_path.cend())
      m_msgs << ",";
  }
  m_msgs << endl;

  return(true);
}




//----------------------------------------------------------
// Procedure: plotPath()

void RoutePlan::plotPath(std::vector<size_t> path)
{
  //  Clear all previous circles. 
  for (vector<XYCircle>::iterator it = m_active_circles.begin(); it != m_active_circles.end(); it++) {
    it->set_active(false);
    Notify("VIEW_CIRCLE", it->get_spec());
    m_active_circles.erase(it--);   
  }

  // exit if no path
  if (path.size() < 1)
    return;
  
  // Find the path color
  std::string path_color;
  if (m_path_color_map.find(m_vname) == m_path_color_map.end()) {
    path_color = "limegreen";
  } else {
    path_color = m_path_color_map.at(m_vname);
  }

  // Draw a different marker if this is the final path
  // that was found
  double radius, edge_size;
  if (m_sent_final_path_found) {
    radius    = 3.0;
    edge_size = 6.0;
  } else {
    radius    = 1.5;
    edge_size = 3.0;
      
  }
 
  // Draw the new path
  for (vector<size_t>::const_iterator it = path.cbegin(); it != path.cend(); ++it) {
    double curr_x = 0;
    double curr_y = 0;
    m_grid.getCellXY(*it, curr_x, curr_y);
    
    XYCircle path_marker(curr_x, curr_y, radius);
    
    path_marker.set_color("edge",path_color);
    path_marker.set_label(std::to_string(*it));
    path_marker.set_edge_size(edge_size);
    Notify("VIEW_CIRCLE", path_marker.get_spec());
    
    m_active_circles.push_back(path_marker);
  }
}

void RoutePlan::plotStartIds()
{
  //  Clear all previous markers 
  for (vector<XYCircle>::iterator it = m_active_start.begin(); it != m_active_start.end(); it++) {
    it->set_active(false);
    Notify("VIEW_CIRCLE", it->get_spec());
    m_active_start.erase(it--);   
  }
  
  // Draw the new markers
  for (set<size_t>::const_iterator it = m_start_ids.cbegin(); it != m_start_ids.cend(); ++it) {
    double curr_x = 0;
    double curr_y = 0;
    m_grid.getCellXY(*it, curr_x, curr_y);

    XYCircle start_marker(curr_x, curr_y, 1.5);
    start_marker.set_label(std::to_string(*it)+"_start");
    start_marker.set_edge_size(3.0);
    
    Notify("VIEW_CIRCLE", start_marker.get_spec());
    
    m_active_start.push_back(start_marker);
  }
}

void RoutePlan::plotEndIds()
{

  //  Clear all previous markers 
  for (vector<XYCircle>::iterator it = m_active_end.begin(); it != m_active_end.end(); it++) {
    it->set_active(false);
    Notify("VIEW_CIRCLE", it->get_spec());
    m_active_end.erase(it--);   
  }
  
  // Draw the new markers
  for (set<size_t>::const_iterator it = m_end_ids.cbegin(); it != m_end_ids.cend(); ++it) {
    double curr_x = 0;
    double curr_y = 0;
    m_grid.getCellXY(*it, curr_x, curr_y);

    XYCircle end_marker(curr_x, curr_y, 1.5);
    end_marker.set_label(std::to_string(*it)+"_end");
    end_marker.set_edge_size(3.0);
    
    Notify("VIEW_CIRCLE", end_marker.get_spec());
    
    m_active_end.push_back(end_marker);

    //cout << *it << " added" << endl;
  }
}

//--------------------------------------------------------
// Procedure updateWaypointPath
void RoutePlan::updateWaypointPath(std::vector<size_t> path)
{
  if (path.size() < 1)
    return;
    
  XYSegList points_out;
  
  for (vector<size_t>::const_iterator it = path.cbegin(); it != path.cend(); ++it) {
    double curr_x = 0;
    double curr_y = 0;
    m_grid.getCellXY(*it, curr_x, curr_y);
    points_out.add_vertex(curr_x, curr_y);
  }
  
  string update_str = "points = ";
  update_str += points_out.get_spec();
  Notify("PATH_UPDATES", update_str);  
  //cout << update_str << endl;
}


//-------------------------------------------------------
// Procedure: handlePathProposal
//            processes a proposed path by another vehicle
//            the message has the following structure
//            vname=abe;1,2,3,4,5,6;123.4
//            where the path is 1,2,3,4,5,6
//                  the cost is 123.4
//                  the vehicle name is vname;
//            return false if error
bool RoutePlan::handlePathProposal(std::string msg, double msg_time)
{

  std::string vname;
  std::vector<std::size_t> proposed_path;
  double cost;
  
  vector<string> svector = parseString(msg, ';');
  // will allways be at least one entry
  if ( biteStringX(svector[0], '=') == "vname") 
    vname = svector[0];
  else
    return(false);

  // Check if this is own message, just to be sure
  // if so, return true without updating anything
  if (vname == m_vname)
    return(true);

  // check length before accessing
  if (svector.size() < 2)
    return(false);

  if (svector[1] == "") {
    return(false);
  } else {

    std::vector<string> path_vstring = parseString(svector[1], ',');
    for(unsigned int i=0; i<path_vstring.size(); i++) {
      bool ok = isNumber(path_vstring[i]);
      if (ok) {
	std::size_t temp_val = static_cast<size_t>( std::stoull(path_vstring[i],nullptr) );
	proposed_path.push_back( temp_val );
      } else {
	return(false);
      }
    }
  }
  
  // check length before accessing
  if (svector.size() < 3)
    return(false);

  if (svector[2] == "" or (not isNumber(svector[2])) )  {
    return(false);
  } else {
    bool ok2 = setDoubleOnString( cost, svector[2]);
    if (not ok2)
      return(false);
  }

  // Everything was successful.  Save these
  m_proposed_paths[vname] = proposed_path;
  m_proposed_costs[vname] = cost;
  m_proposal_arrival_time[vname] = msg_time;
  
  return(true);
}


//------------------------------------------------------
// Procedure getPathProposalSpec
//            vname=abe;1,2,3,4,5,6;123.4
//            where the path is 1,2,3,4,5,6
//                  the cost is 123.4
//                  the vehicle name is vname;
std::string RoutePlan::getPathProposalSpec(std::vector<size_t> path, double cost)
{

  std::string msg = "vname=" + m_vname + ";";
 
  unsigned int length = path.size();
  for (int i = 0; i <length; i++) {
    msg +=  std::to_string(path[i]);
    
    if ( i < (length-1) )
      msg += ",";
  }
  
  msg += ";";
  msg += std::to_string(cost);
  
  return(msg);
}

//-------------------------------------------------
// Procedure checkProposalsForWin()
//           Checks proposals recieved for those that
//           have not expired determine if our proposal
//           is best.
//           Returns true if we have won. 
bool RoutePlan::checkProposalsForWin()
{
  // Some initial checking.  If we don't have a proposal, or if
  // the proposal is old, then exit. 
  if (m_proposed_paths.find(m_vname) == m_proposed_paths.end())
    return(false);

  if (( MOOSTime() - m_proposal_arrival_time.at(m_vname) ) > (m_proposal_wait_time + 1.0)) {
    m_proposal_timeout_cnt++;
    return(false);
  }
  
  bool won_a_proposal = true;
  // All the maps should have all the same keys.  They are all updated
  // together, but there is probably a better way to do this. 
  std::map<std::string,std::vector<std::size_t> >::const_iterator it;
  for (it = m_proposed_paths.begin(); it != m_proposed_paths.end(); it++) {
    // skip if this is our proposal;
    if (it->first == m_vname)
      continue;
  
    // check if this proposal is fresh, or if this is the first proposal we have done,
    // we can include stale proposals.  This is needed when the state transitions from
    // an initial waypoint path to EXPLORE mode, which does not necessarily co-incide with
    // a new consensus grid.  As a result, two vehicles could temporarily have the same
    // path
    bool valid_proposal = ( (MOOSTime() - m_proposal_arrival_time.at(it->first) ) < ( (m_proposal_wait_time * static_cast<double>(m_proposal_iterations)) + 1.0) );
    if (valid_proposal or (not m_first_proposal_completed) ) {
      
      // Is this proposed path the same as the one we proposed?
      bool same_path = (m_proposed_paths.at(m_vname) == it->second);
      if (same_path) {

	// Was this a path that was proposed before we started?
	// i.e. before our first proposal
	if (not m_first_proposal_completed) {
	  // Assume the other vehicle won this proposal a long time ago
	  // we are done
	  won_a_proposal = false;
	  break;
	}
	
	
	// Is the cost for this proposed path better than what it would cost
	// us?  (better than our bidded cost?)
	if ( m_proposed_costs.at(it->first) < m_proposed_costs.at(m_vname)) {

	  // We have been beat by at least one other agent
	  // we are done;
	  won_a_proposal = false;
	  break;	  
	}
      }
    }
   
  } // end of for loop
  
  // update the flag
  m_first_proposal_completed = true;
  
  return(won_a_proposal);
}

//------------------------------------------------------
// Procedure:  getVerticesFromValidPaths()
//             returns a set that contains all the vertices
//             of all the paths found in the valid proposals
//             This set will be empty the first time a new
//             grid is found and there are no valid other proposals
std::set<std::size_t> RoutePlan::getVerticesFromValidPaths() const
{

  std::set<std::size_t> set_to_return;
  
  // All the maps should have all the same keys.  They are all updated
  // together, but there is probably a better way to do this. 
  std::map<std::string,std::vector<std::size_t> >::const_iterator it;
  for (it = m_proposed_paths.begin(); it != m_proposed_paths.end(); it++) {
    // skip if this is our proposal;
    if (it->first == m_vname)
      continue;
    
    // check if this proposal is fresh
    // or we have not won a proposal, in which case we can
    // assume that all the other bids are taken, this can happen
    // if some vehicles start the proposal process well before others
    bool valid_proposal = ( (MOOSTime() - m_proposal_arrival_time.at(it->first)) < ( (m_proposal_wait_time * static_cast<double>(m_proposal_iterations)) + 1.0) );
    if (valid_proposal or (not m_first_proposal_won)) {

      // add all the vectices to the set
      for( unsigned int i=0; i<it->second.size(); i++) {
	set_to_return.insert(it->second[i]);
      }
   
    }
  } // end of for loop
  
  return(set_to_return);
}


//-----------------------------------------------------
//  Procedure Send out own proposal
void RoutePlan::sendOutOwnProposal(std::string spec)
{
  
  NodeMessage node_message;
  node_message.setSourceNode(m_vname);
  node_message.setVarName(m_proposal_var_name);
  node_message.setStringVal(spec);
  node_message.setColor("yellow");
  node_message.setSourceApp("pRoutePlan");
  node_message.setDestNode("all");
  string msg = node_message.getSpec();
  
  Notify("NODE_MESSAGE_LOCAL", msg);
}



// define a cost function
// returns the minimum cost to get to any line between
// any two vertexes 
double RoutePlan::calcCost( const std::vector<double>& v1,
			    const std::vector<std::vector<double> >& V2,
			    std::size_t& index_of_closest_vertex)
{
  double min_distance = std::numeric_limits<double>::max();
  std::vector<double> best_vertex;

  for (int i=0; i<(V2.size() - 1); i++){

    double distance = distPointToSeg(V2[i][0], V2[i][1], V2[i+1][0], V2[i+1][1], v1[0], v1[1]);

    if (distance < min_distance) {
      min_distance = distance;
      // check if the ith vertex is closer than the i+1th vertex
      double dist_i = hypot(V2[i][0] - v1[0], V2[i][1] - v1[1]);
      double dist_i_plus_one = hypot(V2[i+1][0] - v1[0], V2[i+1][1] - v1[1]);
      if (dist_i <= dist_i_plus_one) {
	best_vertex = V2[i];
      } else {
	best_vertex = V2[i+1];
      }
    }
  }

  // Find this vertex in the grid
  int cell_id;
  m_grid.getCellID(best_vertex[0], best_vertex[1], cell_id);
  index_of_closest_vertex = static_cast<std::size_t>(cell_id);
  
  return (min_distance);
}


// -----------------------------------------------------
// Procedure:  findBestPathDirection
//             uses own position and own proposed path
//             to determine the best way to traverse the path
//             The best way has the higher variance. 
std::vector<std::size_t> RoutePlan::findBestPathDirection()
{
  std::vector<std::size_t> empty_path;
      
  // First find the closest index
  std::vector<double> current_position = {m_nav_x, m_nav_y};
  std::vector<std::size_t> own_path = m_proposed_paths.at(m_vname);
  std::vector<std::vector<double> > path_vertices;
  for (unsigned int j=0; j<own_path.size(); j++) {
    path_vertices.push_back(m_vertices.at(own_path[j]));
  }
  std::size_t index_of_closest_vertex; 
  double cost_to_path = calcCost(current_position, path_vertices, index_of_closest_vertex);

  // search both ways down the path and compute metrics to use to
  // determine the best path direction to take.
 
  // find iterator position of the closest index
  std::vector<std::size_t>::iterator it_closest;
  it_closest = std::find(own_path.begin(), own_path.end(), index_of_closest_vertex);
  if (it_closest == own_path.end()) {
    return(empty_path);
  }

  // variables to hold the path and values
  double total_variance_fwd = 0.0;
  int number_of_repeated_vertices_fwd = 0;
  int number_of_start_or_goal_vertices_fwd = 0;
  std::vector<std::size_t> fwd_path;
  
  double total_variance_rev = 0.0;
  int number_of_repeated_vertices_rev = 0;
  int number_of_start_or_goal_vertices_rev = 0;
  std::vector<std::size_t> rev_path;

  // dummy variables that are updated
  // by reference, some are not used
  double x, y, depth, var;
  bool ok;

  // iterate forward through the path
  double discount = 0.5;
  double step_number = 1.0;
  std::vector<std::size_t>::iterator it_fwd;
  for (it_fwd = it_closest; it_fwd != own_path.end(); it_fwd++) {
    ok = m_grid.getCellData(*it_fwd, x, y, depth, var);
    if (not ok)
      return(empty_path);

    double step_var = var * pow(discount,step_number);
    total_variance_fwd += step_var;
    step_number = step_number + 1.0;
    if(std::find(m_p.begin(), m_p.end(), *it_fwd) != m_p.end()){
      number_of_repeated_vertices_fwd++;
    }
    if ( (m_start_ids.count(*it_fwd)>0) or (m_end_ids.count(*it_fwd)>0) ) {
      if(var > m_var_threshold)
	number_of_start_or_goal_vertices_fwd++;
    }
      
    fwd_path.push_back(*it_fwd);
  }

  // iterate backward through the path
  step_number = 1.0;
  std::vector<std::size_t>::iterator it_rev;
  for (it_rev = it_closest; it_rev != (own_path.begin()-1); it_rev--) {
    ok = m_grid.getCellData(*it_rev, x, y, depth, var);
    if (not ok)
      return(empty_path);

    double step_var = var * pow(discount,step_number);
    total_variance_rev += step_var;
    step_number = step_number + 1.0;
    if(std::find(m_p.begin(), m_p.end(), *it_rev) != m_p.end()){
      number_of_repeated_vertices_rev++;
    }
    if ( (m_start_ids.count(*it_rev)>0) or (m_end_ids.count(*it_rev)>0) ) {
      if(var > m_var_threshold)
	number_of_start_or_goal_vertices_rev++;
    }
	
    rev_path.push_back(*it_rev);
  }


  // finally, return the best direction
  // first check  if there are any start or goal vertices,
  // if so then return the path that has the most start or
  // goal vertices
  if ((number_of_start_or_goal_vertices_fwd > 0) or (number_of_start_or_goal_vertices_rev > 0)) {
    cout << "checking direction from # start/goal - forward " << number_of_start_or_goal_vertices_fwd << " reverse " << number_of_start_or_goal_vertices_rev << endl;
    if (number_of_start_or_goal_vertices_fwd > number_of_start_or_goal_vertices_rev) {
      m_rev_path = rev_path;
      return(fwd_path);
    } else if (number_of_start_or_goal_vertices_fwd < number_of_start_or_goal_vertices_rev) {
      m_rev_path = fwd_path;
      return(rev_path);
    }
  }
  
  // Then check if there are any repeated vertices,
  // if so then return the path that has the most repeated
  // indexes

  // If no repeated vertices, return the path that has the most
  // (discounted) variance
  cout << "returning direction from variance ";
  cout << "Variance forward: " << to_string(total_variance_fwd) << " Variance backward: " << to_string(total_variance_rev) << endl;
  if (total_variance_fwd > total_variance_rev) {
    m_rev_path = rev_path;
    return(fwd_path);
  } else {
    m_rev_path = fwd_path;
    return(rev_path);
  }
    
}

//-------------------------------------------------------------
// Procedure:  findInitialExplorePath()
//             Finds all the start or goal nodes (depending
//             on the vehicle number, and orders them
//             in a greedy schedule.
std::vector<size_t> RoutePlan::findInitialExplorePath(){
  
  std::vector<size_t> path;
  XYSegList path_seglist;
  
  std::set<size_t>::iterator it;
  double x, y;

  if ( m_vehicle_n == 0 ) {
    // add all the start cells to a seglist
    for (it = m_start_ids.begin(); it != m_start_ids.end(); it++) {
      m_grid.getCellXY(*it, x, y);
      path_seglist.add_vertex(x, y);     
    }

  } else if ( m_vehicle_n == (m_number_of_vehicles-1) ) {
    // add all the goal cells to a seglist
    for (it = m_end_ids.begin(); it != m_end_ids.end(); it++) {
      m_grid.getCellXY(*it, x, y);
      path_seglist.add_vertex(x, y);     
    }

  } else {
    // we shouldn't be here anyway,
    // return the empty path
    return(path);
  }
  
  // finally, just use greedy scheduling
  // to sort by distance
  path_seglist = greedyPath(path_seglist, m_nav_x, m_nav_y);

  // now convert back to indices.
  int cell_id;
  bool ok;
  for (unsigned int i=0; i<path_seglist.size(); i++) {
    ok = m_grid.getCellID(path_seglist.get_vx(i), path_seglist.get_vy(i), cell_id);
    if (ok) {
      size_t cell_idx = cell_id;   // ensure it is a size_t
      path.push_back(cell_idx);
    } else {
      // error
      return(path);
    }
  }

  return(path);
}


//----------------------------------------
//  Update start or end path if needed.
//         uses vehicle number and total vehicles
//         and if required, finds the path and sends
//         it out. 

void RoutePlan::updateStartOrEndPaths()
{
  bool cond1 = ( ( m_vehicle_n == 0) or (m_vehicle_n == (m_number_of_vehicles-1) ) );
  if (cond1 and m_first_grid_received) {
    std::vector<size_t> initial_path = findInitialExplorePath();
    if (initial_path.size() > 0) {
      updateWaypointPath(initial_path);
    }
  }
  return; 
}


//-----------------------------------------
//  Procedure: calcDynamicThreshold
//             Calculates the threshold for the
//             variance based on the distibution
//             of probabilities

double RoutePlan::calcDynamicThreshold() const
{ 

  double dynam_thresh = 0.5;  // default to return

  // edgecase handling
  std::vector<double> var_vec = m_grid.getVarianceVector();
  if (var_vec.size() <1)  {
    return(dynam_thresh);
  } else if (var_vec.size() <2) {
    return(var_vec[0]);
  }
 
  // just implement the max and min element search to avoid
  // including <algorithm>, and to complete the search
  // in the same loop
  double maxElement = std::numeric_limits<double>::min();
  double minElement = std::numeric_limits<double>::max();

  std::vector<double>::iterator it;
  for(it = var_vec.begin(); it != var_vec.end(); it++ ) {
    if (*it < minElement)
      minElement = *it;
    if (*it > maxElement)
      maxElement = *it;   
  }

  double range = maxElement - minElement;
  dynam_thresh = minElement + m_var_pct_thresh * range;

  return(dynam_thresh);
}

//----------------------------------------
//  Procedure: isViablePath()
//             Returns true if current path does not contain any obstacles        
bool RoutePlan::isViablePath(vector<size_t> path)
{
  if (path.size() <1 )
    return false;
  
  for (vector<size_t>::iterator it = path.begin(); it != path.end(); ++it) {
    if (m_obstacles.count(*it))
      return false;
  }
  return true;
}


//--------------------------------------------
//   Procedure: isPathShorter(path1, path2)
//              Returns true if path1 is shorter than
//              path2, false otherwise.
//              returns true if path2 is empty

bool RoutePlan::isPathShorter(std::vector<size_t> const &path1, std::vector<size_t> const &path2)
{
  double len_path1 = m_grid.getPathLength(path1);

  // if path2 is empty, return true;
  double len_path2 = 0.0;
  if (path2.size() > 0) {
    len_path2 = m_grid.getPathLength(path2);
  } else {
    return(true);
  }
  return( len_path1 < len_path2);
}
