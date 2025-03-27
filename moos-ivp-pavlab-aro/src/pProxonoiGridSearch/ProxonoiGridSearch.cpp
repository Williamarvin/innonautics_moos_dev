/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ProxonoiGridSearch.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ProxonoiGridSearch.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ProxonoiGridSearch::ProxonoiGridSearch()
{
  m_nav_x = 0.0;
  m_nav_y = 0.0;
  m_nav_hdg = 0.0;
  m_sensor_meas = 0.0;
  m_vname = "";
  m_curr_option = "";
  m_tree_depth = 5;
  m_max_mdp_iter = 2;
  m_discount_factor = 0.8;

  m_own_max_value = 0.0;
  m_last_value = 0.0;
  m_no_detection_in_cell_reward = 0.0;
  m_no_detection_in_cell_variance = 1.0;
  m_sensor_meas_variance = 0.1;
  m_timeout_after_sampling = 10;
  m_time_last_sample_finished = MOOSTime(); 

  m_cool_grid_value = 0.1;
  m_cool_grid_variance = 1.0;
  m_cool_grid_interval = 60;
  m_last_time_grid_was_cooled = MOOSTime(); 

  m_valid_option_set.insert("EXPLORE_LIST");
  m_valid_option_set.insert("EXPLOIT_LIST");
  m_valid_option_set.insert("MIGRATE_LIST");

  m_post_grid = false;
  m_grid_posts_skipped = 0;
  m_grid_post_skip_interval = 0;
}

//---------------------------------------------------------
// Destructor

ProxonoiGridSearch::~ProxonoiGridSearch()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ProxonoiGridSearch::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL")) {
      if ( !handleNodeReport(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "NAV_X") {
      m_nav_x = msg.GetDouble();

    } else if (key == "NAV_Y") {
      m_nav_y = msg.GetDouble();

    } else if (key == "NAV_HEADING") {
      m_nav_hdg = msg.GetDouble();

    } else if (key == "NAV_SPEED") {
      m_nav_spd = msg.GetDouble();
      
    } else if (key == "SENSOR_MEAS") {
      m_sensor_meas = msg.GetDouble();

    } else if (key == "CONTACTS_LIST") {
      if ( !handleContactsList(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);

    } else if (key == "OPTION"){
      if (msg.GetString() != ""){
	m_curr_option = toupper(msg.GetString());
      } else
	reportRunWarning("Unhandled Mail: " + key);
    } else if (key == "PROXONOI_REGION"){
      if ( !handleOwnProxPoly(msg.GetString()))
	reportRunWarning("Unhandled Mail: "  + key);
      
    } else if (key == "PROX_POLY_NEIGHBOR"){
      if ( !handleNeighborProxPoly(msg.GetString()))
	reportRunWarning("Unhandled Mail: "  + key);

    } else if (key == "SAMPLE_LOC"){
      if ( !handleSampleNotification(msg.GetString()))
	reportRunWarning("Unhandled Mail: "  + key);
      
    } else if (key == "SAMPLE_FINISHING"){
      if ( !handleSampleFinished(msg.GetString() ))
	reportRunWarning("Unhandled Mail: "  + key);
      
    
    } else if ( m_valid_option_set.count(key) > 0) {
      if ( !handlePopStateList(key, msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
    }
    

  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ProxonoiGridSearch::OnConnectToServer()
{
   //registerVariables();  // commented out to reduce redundancy
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ProxonoiGridSearch::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // update the frozen cells in the grid every time
  m_prox_grid.updateFrozenCells(MOOSTime(), m_timeout_after_sampling);
  
  // check if we are in the grid or not exploring
  int curr_id;
  bool in_grid = m_prox_grid.getCellID(m_nav_x, m_nav_y, curr_id);
  bool exploring = (toupper(m_curr_option) == "EXPLORE");
  if ((!in_grid) || (!exploring)){

    // first clear the seglist on pMarineViewer
    XYSegList seglist_clear;
    seglist_clear.set_label("mdp_path_for_" + m_vname + "_by_" + m_vname);
    seglist_clear.set_active(false); 
    Notify("VIEW_SEGLIST", seglist_clear.get_spec() );

    // Bail now if not in the grid. 
    if (!in_grid){
      AppCastingMOOSApp::PostReport();
      return(true);
    }
  }

  
  // Generate an optimal path using MDP
  // We will update new_path_seglist
  XYSegList new_path_seglist;
  double center_x;
  double center_y;
  
  // get the center of the cell we are in.
  bool ok = m_prox_grid.getCellXY(curr_id, center_x, center_y);
   
  // Determine the best next goal (new_x and new_y)
  double value =0.0;
  getNextPathLookahead(center_x, center_y, m_nav_hdg, m_own_prox_poly, m_own_prox_poly,
		       true, m_vname, new_path_seglist, value);

  
  Notify("MDP_NEXT_NODE_VALUE", m_own_max_value);
  if(false){
  std::string utility_spec; 
  utility_spec +="  Utilities: ";
  std::map<std::string, std::map<std::string,double>>::iterator it_all;
  for (it_all = m_utilities.begin(); it_all != m_utilities.end(); it_all++){
    utility_spec += "  *" + it_all->first + ": ";
    std::map<std::string, double>::iterator it;
    int count = 0;
    int max_count = 30;
    for (it = it_all->second.begin(); it != it_all->second.end(); it++){
      utility_spec += " Cell=" + it->first +",U=" + doubleToString(it->second, 4) + ",";
      if (count >= max_count){
        utility_spec += "!!! ended early   !!!";
	break;
      } else {
	count ++;
      }
    }
    if (count < max_count)
      utility_spec += "!!!    !!!";
  }
  Notify("TMP_DEBUG_UTILS", utility_spec);
  }
  
  // Only update the behavior if the 
  // update waypoint behavior.
  string update_str = "points = ";
  update_str += new_path_seglist.get_spec();
  Notify("PATH_UPDATES", update_str);
  
  new_path_seglist.set_active(true);
  Notify("VIEW_SEGLIST", new_path_seglist.get_spec() );
  
  m_last_value = m_own_max_value;
  

  // get the marginal improvement:
  updateIgnoreSet();
  sliceOwnPolygon();
  double value_with    = calcCollectiveValue(true);
  value_with += m_own_max_value;
  double value_without = calcCollectiveValue(false);
  
  Notify("VALUE_WITH_PARTICIPATION", value_with);
  Notify("VALUE_WITHOUT_PARTICIPATION", value_without);
  
  // Also post grid if configured
  if ((m_post_grid) && ((m_grid_posts_skipped % 1) ==0))
    Notify("VIEW_GRID", m_prox_grid.get_spec());
 
  m_grid_posts_skipped += 1;

  // Cool grid if needed
  if ((MOOSTime() - m_last_time_grid_was_cooled) > m_cool_grid_interval){
    m_prox_grid.coolCellsVisited(m_cool_grid_value, m_cool_grid_variance);
    m_last_time_grid_was_cooled = MOOSTime(); 
  }
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ProxonoiGridSearch::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  std::string grid_config;
  
  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    
    if (param == "grid_config") {
      unsigned int len = grid_config.length();
      if((len > 0) && (grid_config.at(len-1) != ','))
	grid_config += ",";
      grid_config += value;
      handled = true;
    }
    else if(param == "tree_depth") {
      handled = setIntOnString(m_tree_depth, value);
      handled = handled && ( m_tree_depth > 1);
    }
    else if(param == "max_mdp_iter") {
      handled = setIntOnString(m_max_mdp_iter, value);
      handled = handled && ( m_max_mdp_iter > 1); 
    }
    else if(param == "grid_post_skip_interval") {
      handled = setIntOnString(m_grid_post_skip_interval, value);
      handled = handled && ( m_grid_post_skip_interval >= 1); 
    }
    else if(param == "discount_factor") {
      handled = setPosDoubleOnString(m_discount_factor,value);
    }
    else if(param == "post_grid") {
      handled = setBooleanOnString(m_post_grid,value);
    }
    else if(param == "cool_grid_value") {
      handled = setNonNegDoubleOnString(m_cool_grid_value,value);
    }
    else if(param == "cool_grid_variance") {
      handled = setNonNegDoubleOnString(m_cool_grid_variance,value);
    }
    else if(param == "cool_grid_interval") {
      handled = setNonNegDoubleOnString(m_cool_grid_interval,value);
    }
    else if(param == "no_detection_in_cell_reward") {
      handled = setNonNegDoubleOnString(m_no_detection_in_cell_reward,value);
    }
    else if(param == "timeout_after_sampling") {
      handled = setNonNegDoubleOnString(m_timeout_after_sampling,value);
    }
    else if(param == "no_detection_in_cell_variance") {
      handled = setNonNegDoubleOnString(m_no_detection_in_cell_variance,value);
    }
    else if(param == "sensor_meas_variance") {
      handled = setNonNegDoubleOnString(m_sensor_meas_variance,value);
    }
    
    if(!handled)
      reportUnhandledConfigWarning(orig);


  }
  
  // The vehicle name is the host community
  m_vname = m_host_community;

   // initialize grids
  XYConvexGrid temp_grid = string2ConvexGrid(grid_config);
  temp_grid.set_label(m_vname + "'s_grid"); 

  if(temp_grid.size() == 0)
    reportConfigWarning("Unsuccessful ConvexGrid construction.");

  m_prox_grid = ProxGrid(temp_grid, 1.0);

  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ProxonoiGridSearch::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
  Register("CONTACTS_LIST", 0);
  Register("OPTION",0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("PROXONOI_REGION",0);
  Register("PROX_POLY_NEIGHBOR",0);
  Register("SAMPLE_LOC",0);
  Register("SAMPLE_FINISHED",0);
  Register("SAMPLE_FINISHING",0); 
  Register("SENSOR_MEAS",0);

  std::set<std::string>::iterator it;
  for (it=m_valid_option_set.begin(); it!=m_valid_option_set.end(); it++){
    Register(*it,0); 
  }
}


//-----------------------------------------------------
// handleNodeReport(std::string msg)
//
bool ProxonoiGridSearch::handleNodeReport(std::string msg)
{
 
  // process incoming node record 
  NodeRecord newNodeRecord;
  newNodeRecord = string2NodeRecord(msg, true);
  if (!newNodeRecord.valid())
    return(false);

  // Only keep contacts that are alive:
  std::string node_rec_name = tolower(newNodeRecord.getName());
  if ( m_contacts.count(node_rec_name) || (node_rec_name == tolower(m_vname)) )
    m_node_rec_map[node_rec_name] = newNodeRecord;


  // bail now if no one is exploring
  if ((m_pop_state_map.count("EXPLORE") == 0) && (toupper(m_curr_option) != "EXPLORE")){
    return(true);
  }

  // bail now if they are outside the grid
  double x = newNodeRecord.getX();
  double y = newNodeRecord.getY();
  int cell_id = 0;
  bool ok = m_prox_grid.getCellID(x, y, cell_id);
  if (!ok){
    return(true);
  }
  
  // update the grid to reflect their position if they are
  // exploring
  bool neighbor_exploring = m_pop_state_map["EXPLORE"].count(node_rec_name) > 0;
  bool own_record_and_I_am_exploring = (node_rec_name == tolower(m_vname)) && (toupper(m_curr_option) == "EXPLORE");

  bool ok_grid_update = false;
  if (neighbor_exploring){
    // Mark the grid accordingly:
    // with m_cleared_grid_reward, m_cleared_grid_variance variance if
    if (didAgentJustNotifyToSample(node_rec_name,3.0)){
      // they just notified
      ok_grid_update = m_prox_grid.updateCellValueIDX(cell_id, 1.0, 0.1);
      return(ok_grid_update);
    } else {
      // mark as uncertain
      ok_grid_update = m_prox_grid.updateCellValueIDX(cell_id, m_no_detection_in_cell_reward, m_no_detection_in_cell_variance);
      return(ok_grid_update);
    }
    
  } else if (own_record_and_I_am_exploring){
    
    // could check if I notified, but might as well just
    // use the sensor value
    double time_elapsed = MOOSTime() - m_time_last_sample_finished;
    if (time_elapsed > m_timeout_after_sampling/2.0)
      ok_grid_update = m_prox_grid.updateCellValueIDX(cell_id, m_sensor_meas, m_sensor_meas_variance);
    else
      ok_grid_update = true;   // its ok
    
    return(ok_grid_update);
    
  }

  
  return(true);
}



//-----------------------------------------------------
// Procedure: handleContactsList(std::string msg)
//
bool ProxonoiGridSearch::handleContactsList(std::string msg)
{
  // parse message
  std::set<std::string> new_contact_set;
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i=0; i<svector.size(); i++) {
    new_contact_set.insert(tolower(svector[i]));
  }
  m_contacts = new_contact_set;

  // Now clear out any old node reports
  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end();){
    if ( m_contacts.count(tolower(it->first)) || (m_vname == tolower(it->first)) ) {
	++it;
      } else {
	m_node_rec_map.erase(it++);
      }
  }
 
  return(true);
}


// --------------------------------------------------------
// Handle population state updates
//
// builds a set that contains the agents participating in each choice
//                    Key = option name
//                    Val = set of names of agents for that option.
bool ProxonoiGridSearch::handlePopStateList(std::string key, std::string msg){

  // Make a new set to erase the old set.
  std::set<std::string> new_active_set;
  // parse message
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i=0; i<svector.size(); i++) {
    new_active_set.insert(tolower(svector[i]));
  }

  // Some sets could be empty!

  // remove the "_LIST" at the end
  std::string option_name = key.substr(0, key.length()-5);
  option_name = toupper(option_name);

  m_pop_state_map[option_name] = new_active_set; 


  return(true); 
}


//-----------------------------------------------------
// Procedure: handleNeighborProxPoly(std::string msg)
//
bool ProxonoiGridSearch::handleNeighborProxPoly(std::string msg)
{
  XYPolygon new_neighbor_poly = string2Poly(msg);
  if(new_neighbor_poly.active() && !new_neighbor_poly.is_convex())
    return(false);

  // The label should be "vpoly_" + vname
  std::string label = tolower(new_neighbor_poly.get_label());
  std::set<std::string>::iterator it;
  for (it = m_contacts.begin(); it != m_contacts.end(); it++){
    if (label.find(*it) != std::string::npos) {
      // save it in the map
      m_neighbors_prox_poly[*it] = new_neighbor_poly;
      return(true);
    }
  }
  
  return(false);
}



//-----------------------------------------------------
// Procedure: handleOwnProxPoly(std::string msg)
//
bool ProxonoiGridSearch::handleOwnProxPoly(std::string msg)
{
  XYPolygon new_own_poly = string2Poly(msg);
  if(!new_own_poly.is_convex())
    return(false);

  m_own_prox_poly = new_own_poly;
  return(true);
  
}




//-----------------------------------------------------
// Procedure: handleSampleNotifications(std::string msg)
//            SAMPLE_LOC = XYPOINT spec with label"ABE_2_NORTH"
//
bool ProxonoiGridSearch::handleSampleNotification(std::string msg)
{
  XYPoint new_sample_point = string2Point(msg);
  if (!new_sample_point.valid())
    return(false);

  std::string label = new_sample_point.get_label();
  std::string vname = tolower(biteString(label, '_'));
  // Does this label belong to a known contact
  if ((m_contacts.count(vname) == 0) && (vname != m_vname))
    return(false);

  // add it to the map
  m_last_sample_notification_time_map[vname] = MOOSTime(); 
  

  // And clear this grid cell because we already know something is there
  // and so no new information would be gained by going there
  double x = new_sample_point.get_vx();
  double y = new_sample_point.get_vy();
  int cell_id = 0;
  bool ok = m_prox_grid.getCellID(x, y, cell_id);
  if (!ok){
    reportRunWarning("Did not find cell id for : " + msg);
    return(false);
  }

  ok = m_prox_grid.updateCellValueIDX(cell_id, 0.0, m_sensor_meas_variance);
  ok = ok && m_prox_grid.setFrozenCell(cell_id, MOOSTime()); 

  if (!ok)
        reportRunWarning("Did not update cell id for : " + msg);
  return(ok);

}


bool ProxonoiGridSearch::handleSampleFinished(std::string msg)
{

  if (tolower(msg) == m_vname)
    m_time_last_sample_finished = MOOSTime(); 

  // clear the cell with this contacts name
  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end(); it++){
    std::string rec_name = tolower(it->second.getName());
    if (tolower(msg)  == rec_name){

      // now update the grid if they are in it.
      double x = it->second.getX();
      double y = it->second.getY();
      int cell_id = 0;
      bool ok = m_prox_grid.getCellID(x, y, cell_id);
      if (!ok)
	return(false);
      
      ok = m_prox_grid.updateCellValueIDX(cell_id, 0.0, m_sensor_meas_variance);
      return(true); 
    }
  }



  return(true);
}


//-------------------------------------------------
// Procedure: getNextPathLookahead
//            This procedure returns next cell ahead of
//            The vehicle that maximizes value.  
//            Inputs:    curr_x:       x pos of current cell
//                       curr_y:       y pos of current cell
//                       curr_heading: current headging of vehicle
//                       region1:      polygon region1 (see region_combo)
//                       region2:      polygon region2 (see region_combo)
//                       region_combo: if true, path can be in either region1 or region2
//                                     if false, path must be in region1 and not region2
//                       vname:        name of vehicle the path is being generated for
//
//            Outputs:   new_path:   XY seglist as deep as possible until the limit
//                                   is reached.
//                       value:   expected value from selecting this cell

//
//      
bool ProxonoiGridSearch::getNextPathLookahead(double curr_x, double curr_y, double curr_heading,
					      XYPolygon &region1, XYPolygon &region2, bool region_combo,
					      std::string vname, XYSegList &new_path, double &value)
{
  // Build a path tree
  PathTree pathtree;
  buildPathTree(m_tree_depth, curr_x, curr_y, curr_heading, region1, region2, region_combo, pathtree);


  // Check that we found a path
  bool path_tree_empty = pathtree.getRewards().size()==1;
  double best_value =0.0;
  if (!path_tree_empty) {
    new_path = getBestPath(pathtree, vname, best_value);
    
  } else {  // nothing available ahead, turn arround
    int behind_i[] = {0, 0, 0, 0, 0};
    int curr_direction = m_prox_grid.headingToDirection(curr_heading);
    m_prox_grid.getBehindIDs(curr_x, curr_y, curr_direction, behind_i);

    // Find the first cell behind us that is valid
    double back_x = 0.0;
    double back_y = 0.0;
    for (int i=0; i<5; i++) {
      int behind_id = behind_i[i];
      if (behind_id != -1) {
	bool ok = m_prox_grid.getCellXY(behind_id, back_x, back_y);
	if (!ok)
	  return (false);
	XYSegList backup_seglist;
	backup_seglist.add_vertex(back_x, back_y); 
	new_path = backup_seglist; 
      }
    }
  }
  value = best_value;
  return (true) ;
}



// -----------------------------------------------
// Procedure:  buildPathTree
//
// this function looks ahead <iterations> steps,
// and builds a path tree that is later searched
//            Inputs:    itertions:    the depth of the look-ahead tree
//                       curr_x:       x pos of current cell
//                       curr_y:       y pos of current cell
//                       curr_heading: current headging of vehicle
//                       region1:      polygon region1 (see region_combo)
//                       region2:      polygon region2 (see region_combo)
//                       region_combo: if true, path can be in either region1 or region2
//                                     if false, path must be in region1 and not region2
//
//            Outputs:   path:    A graph of all the acceptable states and transitions
//                                given the region specifications


bool ProxonoiGridSearch::buildPathTree(int iterations, double curr_x, double curr_y, double curr_heading,
				       XYPolygon &region1, XYPolygon &region2, bool region_combo, PathTree &path)
{
  // Step 0. Bookkeeping
  // record the number of iterations
  path.setIterations(iterations);

  // preload the zStar function used in the MVI reward if we need
  std::vector<double> values = m_prox_grid.getValueVector();
  std::vector<double> variances = m_prox_grid.getVarianceVector();
  double robot_noise = 0.01;
  std::vector<double> zStar = calculateZStarGumbel(20, values, variances, robot_noise);
  
  
  // Step 1.
  //build root state
  // format is root_id,curr_heading
  int root_id;
  m_prox_grid.getCellID(curr_x, curr_y, root_id);
  int curr_direction = m_prox_grid.headingToDirection(curr_heading);
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
      double temp_val;
      double temp_var;
      m_prox_grid.getCellData(temp_id, temp_x, temp_y, temp_val, temp_var);

      // get the cells ahead of this state
      int ahead_i[3];
      int directions[3];
      m_prox_grid.getAheadIDs(temp_x, temp_y, temp_direction, ahead_i, directions);

      // Step 4. Process children
      // loop through ahead cells if more iterations will be done
      if (i < iterations) {
	for (int j=0; j<3; j++) {

	  
	  // Check if this is a valid cell given two main conditions:
	  // 1) it is in grid 
	  // 2) it meets the region specifications per region1, region2,
	  //    and region_combo
	  bool valid_cell = false;
      
	  bool in_grid = !(ahead_i[j] == -1);
	  
	  bool meets_region_spec = false;
	  
	  double cell_x, cell_y;
	  bool ok = m_prox_grid.getCellXY(ahead_i[j], cell_x, cell_y);
	  if (ok){  // ok might be false if we are outside the grid
	    bool in_region_1 = region1.contains(cell_x, cell_y);
	    bool in_region_2 = region2.contains(cell_x, cell_y);
	    
	    //     region_combo: if true, path can be in either region1 or region2
	    if(region_combo){
	      meets_region_spec = (in_region_1 || in_region_2);
	    } else {
	    //     region_combo: if false, path must be in region1 and not region2
	      meets_region_spec = (in_region_1 && (!in_region_2));
	    }
	  }

	  valid_cell = in_grid && meets_region_spec;
	  
	  if (valid_cell) { // if valid
	    
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
      double reward = getRewardMVI(temp_val, temp_var, zStar);
      std::string zStar_str = "[";

      for (int i=0; i<zStar.size(); i++){
	zStar_str += std::to_string(zStar[i]) + ",";
      }
      //Notify("TMP_DEBUG", "temp_val="+std::to_string(temp_val) +
      //	     " temp_var="+std::to_string(temp_var) +
      //     " zStar="+ zStar_str);
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
//  best path and updates the reward if own path
//  Returns the path as a Seglist

XYSegList ProxonoiGridSearch::getBestPath(PathTree pathtree, std::string vname, double &value)
{
  //////////////////////////////////////
  // MDP value iteration
  double epsilon = 0.1; // value convergence criteria
  SimpleMDP mdp;
  std::map<std::string,double> utilities;
  utilities = mdp.valueIteration(pathtree.getRewards(),
				   pathtree.getEdges(),
				   m_discount_factor,
				   epsilon,
				   m_max_mdp_iter);

  m_utilities[vname] = utilities;
  // Find the best sequence of length m_tree_depth
  std::vector<std::string> best_path_mdp;

  // Start a seglist to display and return;
  XYSegList best_path_seglist;
  best_path_seglist.set_label("mdp_path_for_" + vname + "_by_" + m_vname);
  if (vname == m_vname){
    best_path_seglist.set_color("edge", "white");
    best_path_seglist.set_color("vertex", "limegreen");
  } else {
     best_path_seglist.set_color("edge", "pink");
     best_path_seglist.set_color("vertex", "yellow");
  }
  best_path_seglist.set_vertex_size(8.0);
  best_path_seglist.set_edge_size(2.0);
  best_path_seglist.set_active(true);
  best_path_seglist.add_vertex(m_nav_x, m_nav_y);

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
	double estimated_utility_at_node = utilities[pathtree.getEdges()[i].second];
	if(estimated_utility_at_node >= best_utility_so_far){
	  best_utility_so_far = estimated_utility_at_node;
	  best_next_path_vertex = pathtree.getEdges()[i].second;
	}
      }
    }
    if ((m_vname == vname) && (j==0))
      m_own_max_value = best_utility_so_far;
    if (j==0)
      value = best_utility_so_far;

    // Check if we hit the end
    if(best_next_path_vertex == ""){
      
      // first clear the seglist on pMarineViewer
      //best_path_seglist.set_active(false);
      //Notify("VIEW_SEGLIST", best_path_seglist.get_spec() );
      //best_path_seglist.set_active(true);
      //Notify("VIEW_SEGLIST", best_path_seglist.get_spec() );
      return(best_path_seglist); 
    }

    best_path_mdp.push_back(best_next_path_vertex);
    node_to_expand = best_next_path_vertex;

    // get the x,y value of this state to add to the seglist
    // to return and plot plot for debugging
    std::string cell_index_str = biteStringX(best_next_path_vertex, ',');
    unsigned int cell_index_int = stoul(cell_index_str);
    double x, y;
    bool ok = m_prox_grid.getCellXY(cell_index_int, x, y);
    if (ok)
      best_path_seglist.add_vertex(x,y);
    
  }

  // for debuging
  if (m_vname != vname){
    Notify("VIEW_SEGLIST", best_path_seglist.get_spec() );
    m_alternate_best_paths_for_contacts[vname] = best_path_seglist;
  }
  
  return(best_path_seglist);
}



//-------------------------------------------------
//  Procedure: updateIgnoreSet()
void ProxonoiGridSearch::updateIgnoreSet()
{
  m_ignore_contact_set.clear();
  double cell_size = m_prox_grid.getCellSize();
  double ignore_contact_dist = static_cast<double>(m_tree_depth) * cell_size * 1.0;
  // contacts will be ignored if beyond this distance.  They are so far away that
  // changing own participation will not matter when considering this horizion.

  double cx, cy, dist;
  std::string c_name;
  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end(); it++){
    c_name = tolower(it->second.getName());
    cx = it->second.getX();
    cy = it->second.getY();
    dist = distPointToPoint(cx, cy, m_nav_x, m_nav_y);

    if (dist > ignore_contact_dist){
      m_ignore_contact_set.insert(c_name);
      Notify("PSG_DEBUG", "Distance for " + c_name + " = " + std::to_string(dist) + " which is more than " + std::to_string(ignore_contact_dist)); 
      continue;
    } 

    // check if they are exploring
    if (m_pop_state_map.count("EXPLORE") > 0){
      bool neighbor_exploring = m_pop_state_map["EXPLORE"].count(c_name) > 0;
      if (!neighbor_exploring){
	m_ignore_contact_set.insert(c_name);
	Notify("PSG_DEBUG", "This neighbor is not exploring-> " + c_name); 
	continue;
      }
    }
    
    // check if they are in the grid
    int curr_id;
    bool in_grid = m_prox_grid.getCellID(cx, cy, curr_id);
    if (!in_grid){
      m_ignore_contact_set.insert(c_name);
      Notify("PSG_DEBUG", "Did not find a polygon for " + c_name); 
      continue;
    }

    // check if we have a poly for them
    if (m_neighbors_prox_poly.count(c_name) == 0){
      m_ignore_contact_set.insert(c_name);
      Notify("PSG_DEBUG", "Did not find a polygon for " + c_name); 
      continue;
    }

 
  }

  // clear any seglists we may have posted.
  std::map<std::string, XYSegList>::iterator sit;
  for (sit=m_alternate_best_paths_for_contacts.begin(); sit != m_alternate_best_paths_for_contacts.end();){
    if (m_ignore_contact_set.count(sit->first) > 0){
      sit->second.set_active(false);
      Notify("VIEW_SEGLIST", sit->second.get_spec());
      sit = m_alternate_best_paths_for_contacts.erase(sit);
    } else
      ++sit;
  }

  std::string ignore_list_str = "";
  std::set<std::string>::iterator it2;
  for (it2 = m_ignore_contact_set.begin(); it2 != m_ignore_contact_set.end(); it2++){
    ignore_list_str += *it2 + ",";
  }
  Notify("PSG_DEBUG_IGNORE", ignore_list_str);
  
  return;
}


//-------------------------------------------------
//  Procedure: calcValueWithOwnParticipation()
double ProxonoiGridSearch::calcCollectiveValue(bool assume_participation)
{

  double total_value = 0.0; 
  
  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end(); it++){
     std::string contact_name = tolower(it->second.getName());
     double c_x = it->second.getX();
     double c_y = it->second.getY();
     double c_hdg = it->second.getHeading(); 

     if (m_ignore_contact_set.count(contact_name))
       continue;
     
     // Generate an optimal path using MDP
     // We will update the new_x and new_y
     XYSegList new_seglist;  // not really used here
     double value;
     
     // get the center of the cell we are in.
     int curr_id;
     double center_x;
     double center_y;
     bool in_grid = m_prox_grid.getCellID(c_x, c_y, curr_id); // already checked 
     bool ok = m_prox_grid.getCellXY(curr_id, center_x, center_y);

     XYPolygon region1 = m_neighbors_prox_poly[contact_name];  // already checked
     XYPolygon region2;
     bool region_combo;

     determineRegionConfig(region2, region_combo,
			   assume_participation, contact_name);
     
     // Determine the best next goal (new_x and new_y)
     getNextPathLookahead(center_x, center_y, c_hdg, region1, region2,
			  region_combo, contact_name, new_seglist, value);

     total_value = total_value + value;
  }  
  return(total_value);
}



// ----------------------------------------------------------
// Determines the apporopriate second region to use in the
// path search, as well as the boolean value for region_combo
//        Inputs:    assume_participation:  true if own particip is assumed
//                   contact_name:   contact_name

void ProxonoiGridSearch::determineRegionConfig(XYPolygon &region2, bool &region_combo,
					       bool assume_participation, std::string contact_name)
{
  // need to set the other poly and combo logic.
  bool actually_participating = (toupper(m_curr_option) == "EXPLORE");

  if (assume_participation){

    // There are two possiblites:
    // 1) We are exploring, so our ploy and our neighbors polys are in sync,
    // 2) We are not exploring, so our neighbors polys will overlap our own.
    if (actually_participating){
      // everything is in sync, just reuse the same poly
      region2 = m_neighbors_prox_poly[tolower(contact_name)]; // assumed already checked if exists.
      region_combo = true;  // can be in either region since the regions are the same
    } else {
      // things are not in sync. The path must be within the neighbors current
      // poly, and not within our own poly that we would be exploring if
      // we were actually participating
      region2 = m_own_prox_poly;
      region_combo = false;
    }
  } else {
    // do not assume we are participating
    // There are two possiblites:
    // 1) We are exploring, so our ploy and our neighbors polys are in sync,
    // 2) We are not exploring, so our neighbors polys will overlap our own.
    if (actually_participating){
      
      // everything is not in sync, so our own poly needs to be sliced up as
      // if we were not participating.
      if (m_own_poly_sliced_pieces.count(contact_name) > 0){
	region2 = m_own_poly_sliced_pieces[contact_name]; // assumed already checked if exists.
	region_combo = true;  // can be in either contact's own poly or this sliced up poly.
      } else {
	// something didn't work out when slicing own polygon, just use the contacts region again.
	region2 = m_neighbors_prox_poly[tolower(contact_name)];  // assumed already checked if exists.
	region_combo = true; // can be in either region since the regions are the same
      }
    } else {
      // everything is in sync, the neighbors polys reflect the assumption
      // that I am not participating
      region2 = m_neighbors_prox_poly[tolower(contact_name)];  // assumed already checked if exists.
      region_combo = true; // can be in either region since the regions are the same
    }
  }
  return;
}


//------------------------------------------------------------
// Procedure: sliceOwnPolygon()
//            cuts up own poly and updates m_own_poly_sliced
void ProxonoiGridSearch::sliceOwnPolygon()
{
  m_own_poly_sliced_pieces.clear();

  // for each contact, generate a new polygon that is a slice of
  // own proxonoi region.  This requires looping through each contact and
  // generating a slice that considers all the other contacts

  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end(); it++){
     std::string contact_name = tolower(it->second.getName());
     double c_x = it->second.getX();
     double c_y = it->second.getY();

     if (m_ignore_contact_set.count(contact_name))
       continue;

     // start with the entire poly
     XYPolygon this_contacts_slice = m_own_prox_poly;

     // Slice it up
     std::map<std::string, NodeRecord>::iterator it2;
     for (it2 = m_node_rec_map.begin(); it2 != m_node_rec_map.end(); it2++){
       std::string contact2_name = tolower(it2->second.getName());
       double c2_x = it2->second.getX();
       double c2_y = it2->second.getY();

       // is this the same contact
       if (contact_name == contact2_name)
	 continue;
       
       if (m_ignore_contact_set.count(contact2_name) > 0)
	 continue;

       // Step 1.  calcuate split line with this contact
       
       // From pProxonoi update split lines (Mike B)
       // If the contact is in the op_region, then create a split line
       // otherwise the splitline associated with the contaxt is null
       XYSegList segl;
       double sx1,sy1,sx2,sy2;
       vsplit(c_x, c_y, c2_x, c2_y, sx1, sy1, sx2, sy2);
       segl.add_vertex(sx1, sy1);
       segl.add_vertex(sx2, sy2);

       // Step 2. Find a point just inside own poly that we can
       // use as an orientation point.
       XYPoint contact_point(c_x,c_y);
       XYPoint orientation_point = this_contacts_slice.closest_point_on_poly(contact_point);
       XYPoint centroid_point = this_contacts_slice.get_centroid_pt();
       
       // move the point slightly towards the centroid and inside the
       // polygon
       double angle = relAng(orientation_point, centroid_point);
       int count = 0;
       int max_count = 1000;
       while (!this_contacts_slice.contains(orientation_point) && (count < max_count)){
	 orientation_point = projectPoint(angle, 0.01, orientation_point);
	 count++;
       }
       if (count == max_count)
	 continue;

       // Step 3. chop the polygon with this split line
       this_contacts_slice = polychop(this_contacts_slice, orientation_point.get_vx(),
				      orientation_point.get_vy(), segl);

     }
     // Now we have a polygon for this contact that is sliced up
     m_own_poly_sliced_pieces[contact_name] = this_contacts_slice;
  }
  
  return;
}



//------------------------------------------------------------
// Procedure: didAgentJustNotifyToSample()

bool ProxonoiGridSearch::didAgentJustNotifyToSample(std::string vname, double stale_thresh) 
{
  // Check if in map
  if (m_last_sample_notification_time_map.count(vname) == 0){
    return(false);
  }

  double rec_time = m_last_sample_notification_time_map[vname];
  double dt = MOOSTime() - rec_time;
  if (dt < 0.0)
    dt *= -1.0;

  if (dt < stale_thresh)
    return(true);
  else 
    return(false); 
}





//------------------------------------------------------------
// Procedure: buildReport()

bool ProxonoiGridSearch::buildReport() 
{
  m_msgs << "============================================" << endl; 
  m_msgs << "                                            " << endl;
  m_msgs << "=  Node Reports   ==========================================" << endl;
  m_msgs << "===========================================================" << endl;
  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end(); it++){
    m_msgs << " <- " << it->second.getSpec() << endl;
  }

    
  m_msgs << "  Utilities: " << endl;
  std::map<std::string, std::map<std::string,double>>::iterator it_all;
  for (it_all = m_utilities.begin(); it_all != m_utilities.end(); it_all++){
    m_msgs << "  *" << it_all->first << ": ";
    std::map<std::string, double>::iterator it;
    int count = 0;
    int max_count = 3;
    for (it = it_all->second.begin(); it != it_all->second.end(); it++){
      m_msgs << " Cell=" << it->first << ",U=" << doubleToString(it->second, 4) << ",";
      if (count >= max_count){
	m_msgs << endl;
	break;
      } else {
	count ++;
      }
    }
    if (count < max_count)
      m_msgs << endl;
  }


  m_msgs << "                                               " << endl;
  m_msgs << " Own poly sliced pieces:                       " << endl;
  std::map<std::string, XYPolygon>::iterator it2;
  for (it2 = m_own_poly_sliced_pieces.begin(); it2 != m_own_poly_sliced_pieces.end(); it2++){
    m_msgs << " *" << it2->first << ": " << it2->second.get_spec() << endl;
  }

	 /*
  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();
  */

  return(true);
}






