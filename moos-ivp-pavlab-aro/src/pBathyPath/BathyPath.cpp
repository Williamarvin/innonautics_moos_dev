/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BathyPath.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "BathyPath.h"
#include "XYSegList.h"
#include "XYFormatUtilsConvexGrid.h"
#include "unistd.h"
#include "EsriBathyGridUtils.h"


using namespace std;

//---------------------------------------------------------
// Constructor

BathyPath::BathyPath()
{
}

//---------------------------------------------------------
// Destructor

BathyPath::~BathyPath()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool BathyPath::OnNewMail(MOOSMSG_LIST &NewMail)
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
                
      bool cond = (label == "cons"); // Shouldn't be any other label...
      if (cond) {
	// update m_grid
	XYConvexGrid temp_grid = string2ConvexGrid(sval);
	m_grid.setXYConvexGrid(temp_grid);
	cout << "GRID RECEIVED" << endl;

	if (!m_first_grid_received)
	  m_first_grid_received = true;

	m_full_grid_rcvd++;
	
      } else {
	reportRunWarning("Mail Error: Grid received from VIEW_GRID_CONS_LOCAL was not labeled 'cons'");
      }
      
    } else if (key == "VIEW_GRID_CONS_LOCAL_DELTA") {
      if (!m_first_grid_received)
	reportRunWarning("Mail Error: Received a grid delta message before receiving the full grid.");
      
      bool ok = m_grid.processGridDelta(sval);
      if (not ok)
	reportRunWarning("Mail Error: Did not process grid delta correctly");
      m_grid_deltas_rcvd++;
 
    } else if (key == "NAV_X") {    
      m_x = dval;
    } else if (key == "NAV_Y") {
      m_y = dval;
    } else if (key == "NAV_HEADING") {
      m_heading = dval;
    } else if (key == "SURVEYING") {
      m_surveying = (sval == "true");
    } else if (key == "PATH_FOUND") {
      if (isBoolean(sval)) {
	m_path_found = (tolower(sval) == "true");

	if (m_path_found) {
	  // clear the seglist on pMarineViewer
	   XYSegList best_path_seglist;
	   best_path_seglist.set_label("mdp");
	   best_path_seglist.set_active(false);
	   Notify("VIEW_SEGLIST", best_path_seglist.get_spec() );
	} else {
	  // set flag to start again if a path was
	  // not found
	  m_new_id = -1;
	}
	  
      } else {
	reportRunWarning("Mail Error: PATH_FOUND value is not boolean");
      }
      
    } else if (key != "APPCAST_REQ") { // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }

  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool BathyPath::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool BathyPath::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  // Bail right away if not surveying
  // or a path is found
  if (!m_surveying or m_path_found) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  
  if (!m_lawnmower) {
    // Generate a path using MDP
    double new_x;
    double new_y;
    double center_x;
    double center_y;
    int curr_id = -1;

    // check if we are in the grid and also get the id, x, and y
    // position of the cell we are in.
    if (m_grid.getCellCenter(m_x, m_y, curr_id, center_x, center_y)) { // in the grid

      // check if we have reached our goal
      bool we_just_started = (m_new_id == -1);
      bool we_reached_last_goal = (curr_id == m_new_id);
    
      if (we_just_started or we_reached_last_goal ) {
    
	// Determine the best next goal (new_x and new_y)
	getNextCellLookahead(center_x, center_y, m_heading, new_x, new_y, m_new_id);

	// increment the number of rounds
	m_round_number = m_round_number + 1.0;
	
	// update waypoint behavior. 
	XYSegList points_out;
	points_out.add_vertex(new_x, new_y); 
	string update_str = "points = ";
	update_str += points_out.get_spec();
	Notify("PATH_UPDATES", update_str);

      } else {
	cout << "Have not yet reached goal" << endl;
      }
    
    } else {
      cout << "not in grid" << endl;
    }
    
  } else {
    // Generate lawnmower path
    
    m_lawnstring = getLawnmowerString2(m_grid, m_vnum, m_vN);
      
    // update waypoint behavior. 
    string update_str = "points = ";
    update_str += m_lawnstring;
    Notify("PATH_UPDATES", update_str);

    
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool BathyPath::OnStartUp()
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
    if (param == "tree_depth") {
      m_tree_depth = stoi(value.c_str());
      handled = isNumber(value);
    }
    else if (param == "use_ucb_for_reward") {
      if (value == "1") {
	m_use_UCB_reward = true;
      } else {
	m_use_UCB_reward = false;
      }
      handled = true;
    }
    else if (param == "prob_scale_factor") {
      m_prob_scale_factor = stod(value.c_str() );
      handled = isNumber(value);
    }
    else if (param == "number_of_vehicles") {
      m_vnum = stoi(value);
      handled = isNumber(value);
    }
    else if (param == "vehicle_n") {
      m_vN = stoi(value);
      handled = isNumber(value);
    }
    else if (param == "grid_degrees") {
      m_degrees = stoi(value);
      handled = isNumber(value);
    }
    else if (param == "lawnmower") {
      m_lawnmower = (value == "1");
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

  XYConvexGrid temp_grid;
  m_grid = EsriBathyGrid(temp_grid,m_lat_origin,m_lon_origin,m_no_data_value);
  
  registerVariables();	
  return(true);
}



// -----------------------------------------------
// Procedure:  buildPathTree
//
// this function looks ahead <iterations> steps,
// and builds a path tree that is later searched
bool BathyPath::buildPathTree(int iterations, double curr_x, double curr_y, double curr_heading, PathTree &path)
{
  // Step 0. Bookkeeping
  // record the number of iterations
  path.setIterations(iterations);

  // preload the zStar function used in the MVI reward if we need
  std::vector<double> zStar;
  if (not m_use_UCB_reward){
    std::vector<double> depths = m_grid.getDepthVector();
    std::vector<double> variances = m_grid.getVarianceVector();
    
    // scale the vector the simple way to avoid linking to algo lib
    for (int i=1; i<variances.size();++i){
      variances[i] = variances[i] * m_prob_scale_factor;
    }
    
    zStar = calculateZStarGumbel(20, depths, variances, m_robot_noise);
  }

  // Get the average depth of the grid to use in the reward
  double average_depth = m_grid.getGridAverageDepth();

  // Step 1.
  //build root state
  // format is root_id,curr_heading
  int root_id;
  m_grid.getCellID(curr_x, curr_y, root_id);
  int curr_direction = m_grid.headingToDirection(curr_heading);
  string root_state = to_string(root_id) + "," + to_string(curr_direction); 
  path.setRootState(root_state); 


  // Step 2. iterate through each next brach
  // vector of cells to process in the
  // search algorithm below
  vector<string> cells_to_process;
  cells_to_process.push_back(root_state);  

  for (int i=0; i<=iterations; i++) {

    // these cells_are the next_to_process
    vector<string> next_cells_to_process;

    vector<string>::iterator it = cells_to_process.begin();
    while (it != cells_to_process.end()) {

      // Step 3.  Expand the current state
      // access current element and convert to int
      string state = *it;
      vector<string> str_vector = parseString(state, ',');
      int temp_id = stoi(str_vector[0]);
      int temp_direction = stoi(str_vector[1]);

      // pull out element data of this cell
      double temp_x;
      double temp_y;
      double temp_depth;
      double temp_var;
      double temp_slope;
      m_grid.getCellData(temp_id, temp_x, temp_y, temp_depth, temp_var);

      // get the cells ahead of this state
      int ahead_i[3];
      int directions[3];
      m_grid.getAheadIDs(temp_x, temp_y, temp_direction, ahead_i, directions);

      // Step 4. Process children
      // loop through ahead cells if more iterations will be done
      if (i < iterations) {
	for (int j=0; j<3; j++) {
	  
	  if (ahead_i[j] != -1) { // if not out of bounds
	    
	    string ahead_state = to_string(ahead_i[j]) + "," + to_string(directions[j]);
	    
	    // add to next processing queue if more iterations will be done
	    next_cells_to_process.push_back(ahead_state);
	    
	    // add edge
	    std::pair<std::string,std::string> edge;
	    edge.first = state;
	    edge.second = ahead_state;
	    path.addEdge(edge);
	    
	  }
	}
      }

      // Step 5.  Calculate reward
      // put it in the reward map
      double reward;
      if (m_use_UCB_reward) {
  
	double delta = 0.1;
	double n_cells = static_cast<double>(m_grid.size());
	reward = getRewardUCB(temp_depth, temp_var*m_prob_scale_factor, n_cells,
			      m_round_number, delta);
      } else {
	reward = getRewardMVI(temp_depth, temp_var*m_prob_scale_factor, zStar);
      }
      path.addReward(state, reward);

      // remove once done processing
      it = cells_to_process.erase(it);
    }
    
    // load the cells_to_process for the next iteration
    cells_to_process = next_cells_to_process;   
  }

  return true;
}


//-----------------------------------------------
//  Proceedure:  getBestPath
//  This function searches the tree and gets the
//  best path. 
string BathyPath::getBestPath(PathTree pathtree)
{
  //////////////////////////////////////
  // MDP value iteration
  double discount_MDP = 0.9;
  double epsilon = 0.1;
  int max_count = 50;
  SimpleMDP mdp;
  m_utilities = mdp.valueIteration(pathtree.getRewards(),
				   pathtree.getEdges(),
				   discount_MDP,
				   epsilon,
				   max_count);

  // Find the best sequence of length m_tree_depth
  std::vector<std::string> best_path_mdp;

  // Start a seglist to display. 
  XYSegList best_path_seglist;
  best_path_seglist.set_label("mdp");
  best_path_seglist.set_color("edge", "white");
  best_path_seglist.set_color("vertex", "limegreen");
  best_path_seglist.set_vertex_size(8.0);
  best_path_seglist.set_edge_size(2.0);
  best_path_seglist.set_active(true);
  best_path_seglist.add_vertex(m_x,m_y);

  // add the root state
  std::string node_to_expand = pathtree.getRootState();
  best_path_mdp.push_back(pathtree.getRootState() );

  // find all the next states
  for (int j=0; j<m_tree_depth; j++) {
    
    // find the best utility possible in the next state
    double best_utility_so_far = 0.0;
    std::string best_next_path_vertex = "";
    for (int i=0; i<pathtree.getEdges().size(); i++) {
      if (pathtree.getEdges()[i].first == node_to_expand) {
	if(pathtree.getRewards()[pathtree.getEdges()[i].second] >= best_utility_so_far){
	  best_utility_so_far = pathtree.getRewards()[pathtree.getEdges()[i].second];
	  best_next_path_vertex = pathtree.getEdges()[i].second;
	}
      }
    }

    // Check if we hit the end
    if(best_next_path_vertex == ""){
      // just return the best path we have for now, it might
      // be the root node if j =0
      
      // first clear the seglist on pMarineViewer
      best_path_seglist.set_active(false);
      Notify("VIEW_SEGLIST", best_path_seglist.get_spec() );
      best_path_seglist.set_active(true);
      Notify("VIEW_SEGLIST", best_path_seglist.get_spec() );

      // check if we hit the end unexpectedly
      if (j == 0) { // no other step was found
	return(best_path_mdp[0]);  
       } else if (j >0) { // j is incremented below, j>0 means at least one step was found
	Notify("VIEW_SEGLIST", best_path_seglist.get_spec() );
      	return(best_path_mdp[1]);  //  
       }
    }

    best_path_mdp.push_back(best_next_path_vertex);
    node_to_expand = best_next_path_vertex;

    // get the x,y value of this state to plot for debugging
    std::string cell_index_str = biteStringX(best_next_path_vertex, ',');
    unsigned int cell_index_int = stoul(cell_index_str);
    double x, y;
    bool ok = m_grid.getCellXY(cell_index_int, x, y);
    if (ok)
      best_path_seglist.add_vertex(x,y);
    
  }


  Notify("VIEW_SEGLIST", best_path_seglist.get_spec() );
  std::string best_next_state = best_path_mdp[1];
  return(best_next_state);
}


//-------------------------------------------------
// Procedure: getNextCellLookahead
//            This procedure returns next cell ahead of the
//      
bool BathyPath::getNextCellLookahead(double curr_x, double curr_y, double curr_heading, double &new_x, double &new_y, int &new_id)
{

  // Build a path tree
  PathTree pathtree;
  buildPathTree(m_tree_depth, curr_x, curr_y, curr_heading, pathtree);


  // Check that we found a path
  bool path_tree_empty = pathtree.getRewards().size()==1;

  if (!path_tree_empty) {
    string st = getBestPath(pathtree);
    vector<string> str_vector = parseString(st, ',');  
    new_id = stoi(str_vector[0]);
    
  } else {  // nothing available ahead, turn arround
    int behind_i[] = {0, 0, 0, 0, 0};
    int curr_direction = m_grid.headingToDirection(curr_heading);
    
    m_grid.getBehindIDs(curr_x, curr_y, curr_direction, behind_i);

    // Find the first cell behind us that is valid
    for (int i=0; i<5; i++) {  
      if (behind_i[i] != -1) {
	new_id=behind_i[i];
	break;
      }
    }
  }

  return ( m_grid.getCellXY(new_id, new_x, new_y) ) ;
}


//---------------------------------------------------------
// Procedure: registerVariables

void BathyPath::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VIEW_GRID_CONS_LOCAL", 0);
  Register("VIEW_GRID_CONS_LOCAL_DELTA", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("SURVEYING", 0);
  Register("PATH_FOUND",0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool BathyPath::buildReport() 
{
  m_msgs << "Surveying = " << boolToString(m_surveying) << endl;
  m_msgs << "Number of full grids received = " << m_full_grid_rcvd << endl;
  m_msgs << "Number of grid deltas recieved = " << m_grid_deltas_rcvd << endl;
  m_msgs << "Average depth of grid = " << doubleToString(m_grid.getGridAverageDepth(), 3) << endl;
  m_msgs << "Maximum grid variance (adjusted) " << doubleToString(m_prob_scale_factor*m_grid.getMaxVar(), 4) << endl;
  m_msgs << "Path found from pRoutePlan = " << boolToString(m_path_found) << endl;
  m_msgs << "------------------------------------------------" << endl;
  if (m_use_UCB_reward)
    m_msgs << "Using Upper Confidence Bound (UCB) reward" << endl;
  else
    m_msgs << "Using Maximum-value Information (MVI) Heuristic Reward" << endl;
  m_msgs << "------------------------------------------------" << endl;
  m_msgs << "\n-- Running MDP = " << boolToString( not (!m_surveying or m_path_found)) << endl;
  m_msgs << "  Depth of search = " << m_tree_depth << endl;
  m_msgs << "  Next cell id " << m_new_id << endl;
  /*
  m_msgs << "  Utilities: " << endl;
  std::map<std::string, double>::iterator it;
  int count = 0;
  int max_count = 3;
  for (it = m_utilities.begin(); it != m_utilities.end(); it++){
    m_msgs << "Cell=" << it->first << ",U=" << doubleToString(it->second, 10) << ",";
    if (count >= max_count){
      m_msgs << endl;
      count = 0;
    } else {
      count ++;
    }
  }
  m_msgs << endl;
  */
  m_msgs << "\n-------------------------------------" << endl;
  m_msgs << "Lawnmower string = " << m_lawnstring << endl;
  m_msgs << "Grid spec = " << m_grid.getGridSpec() << endl;
													  
  return(true);
}




