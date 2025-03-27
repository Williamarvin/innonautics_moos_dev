/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GridSwitcher.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "GridSwitcher.h"
#include "EsriBathyGridUtils.h"
#include "NodeMessage.h"


using namespace std;

//---------------------------------------------------------
// Constructor

GridSwitcher::GridSwitcher()
{
 
  m_recalc_gt_grid = true;
  m_current_grid_label = "psg";
  m_output_var = "";
  m_switching_var = "";
  m_gt_grid_filename = "";
  m_label_switched = false;
  m_want_variance = true;
}

//---------------------------------------------------------
// Destructor

GridSwitcher::~GridSwitcher()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool GridSwitcher::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval  = msg.GetString();
    
#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
    
    if(key == m_switching_var) {
      // handle special case of consensus var
      if (sval == "cons_var") {
	m_current_grid_label = "cons";
	m_want_variance = true;
      } else if (sval == "cons") {
	m_current_grid_label = sval;
	m_want_variance = false;
      } else {
	m_current_grid_label = sval;
      }
      m_label_switched = true;
      
    } else if (m_input_vars.count(key) >= 1) {
      bool ok1 = handleFullGridMsg(sval);
      if (not ok1)
	reportRunWarning("Unhandled Mail: Error reading full grid message");

    } else if (m_input_vars_delta.count(key) >= 1) {
      bool ok2 = handleDeltaGridMsg(sval);
      if (not ok2)
	reportRunWarning("Unhandled Mail: Error reading delta grid message");

    } else if (key == "ROUTE_FOUND" or key == "ROUTE_FOUND_OTHER") {

      if (sval == "1") {

	string path_str = "";
	string cons_depth_str = "";
	string cons_var_str = "";

	// get each vector
	if (m_grids_received.count("psg"))
	  path_str = m_grids_received.at("psg").getPathVectorString();
	if (m_grids_received.count("cons")) {
	  cons_depth_str = m_grids_received.at("cons").getDepthVectorString();
	  cons_var_str = m_grids_received.at("cons").getVarianceVectorString();
	}
	
	// post to DB
	Notify("FINAL_CONSENSUS_DEPTH", cons_depth_str);
	Notify("FINAL_CONSENSUS_VARIANCE", cons_var_str);
	Notify("PATH_VECTOR", path_str);

	Notify("COMPLETED", m_vname);

	NodeMessage node_message;
	node_message.setSourceNode(m_vname);
	node_message.setVarName("ROUTE_FOUND_OTHER");
	node_message.setStringVal("1");
	node_message.setColor("blue");
	node_message.setSourceApp("pGridSwitcher");
	node_message.setDestNode("all");
	string msg = node_message.getSpec();
	  
	Notify("NODE_MESSAGE_LOCAL", msg);
      
      }
      
    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  
  bool vnameOK = m_MissionReader.GetValue("Community", m_vname);
  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool GridSwitcher::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GridSwitcher::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Part 1.  Send a grid in response to a button click. 
  if(m_label_switched) {
    //check if we have this grid, and return if not.
    if (m_grids_received.find(m_current_grid_label) == m_grids_received.end() ) {
      AppCastingMOOSApp::PostReport();
      return(true);
    }

    // Need to send the full spec because the entire grid must be redrawn
    // First check if this grid has some deltas we can update first
    if (m_need_to_update_grid_delta[m_current_grid_label]) {
      m_grids_received.at(m_current_grid_label).setOldGridToNew();
      m_need_to_update_grid_delta[m_current_grid_label] = false; 
    }
    // Then send out the full spec
    std::string grid_spec;
    if ( ( m_current_grid_label == "cons" ) and ( m_want_variance ) ) {
      grid_spec = m_grids_received.at(m_current_grid_label).getVarianceGridSpec();
    } else {
      grid_spec = m_grids_received.at(m_current_grid_label).getGridSpec();
    }
    std::string relabeled_grid_spec = replaceGridLabel(grid_spec, m_vname);
    Notify(m_output_var, relabeled_grid_spec);
    m_need_to_update_full_grid[m_current_grid_label] = false;

    m_label_switched = false;
    
  } else {
    
    // Part 2.  Check if the grid currently displayed needs to be updated with
    //          new info that arrived after the grid was switched. 
    
    // Do we need to send a delta or the whole grid?
    if (m_need_to_update_full_grid[m_current_grid_label]) {
      // Just send the entire new spec.
      std::string grid_spec;
      
      if ( ( m_current_grid_label == "cons" ) and ( m_want_variance) ) {
	grid_spec = m_grids_received.at(m_current_grid_label).getVarianceGridSpec();
      } else {
	grid_spec = m_grids_received.at(m_current_grid_label).getGridSpec();
      }
      std::string relabeled_grid_spec = replaceGridLabel(grid_spec, m_vname);
      Notify(m_output_var, relabeled_grid_spec);
      m_need_to_update_full_grid[m_current_grid_label] = false;
      m_need_to_update_grid_delta[m_current_grid_label] = false;  // just in case

    } else if (m_need_to_update_grid_delta[m_current_grid_label]) {
      
      // First handle the special case of when the variance grid is being shown
      // and a new consensus grid delta is recieved.  Just send a new
      // grid
      if ( ( m_current_grid_label == "cons" ) and ( m_want_variance) ) {
	m_grids_received.at(m_current_grid_label).setOldGridToNew();
	std::string var_grid_spec = m_grids_received.at(m_current_grid_label).getVarianceGridSpec();
	std::string relabeled_grid_spec = replaceGridLabel(var_grid_spec, m_vname);
	Notify(m_output_var, relabeled_grid_spec);
	m_need_to_update_full_grid[m_current_grid_label] = false;
	m_need_to_update_grid_delta[m_current_grid_label] = false;
	
      } else {
	// Just send the delta if there is any
	bool found_delta = false;
	std::string delta_grid_spec = m_grids_received.at(m_current_grid_label).getDeltaSpec(0.01, found_delta);
	
	if (found_delta) {
	  // Replace the label to be the vehicle name
	  XYGridUpdate delta_grid_update = stringToGridUpdate(delta_grid_spec);
	  delta_grid_update.setGridName(m_vname);
	  std::string relabeled_delta_spec = delta_grid_update.get_spec();
	  
	  Notify( (m_output_var + "_DELTA"), relabeled_delta_spec);
	  m_grids_received.at(m_current_grid_label).setOldGridToNew();
	}
	m_need_to_update_grid_delta[m_current_grid_label] = false;
      }
    }
    
  }  // End-if Button click

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GridSwitcher::OnStartUp()
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
    if(param == "input_grid_vars") {
      handled = handleConfigInputVars(toupper(value));
    }
    else if(param == "output_var") {
      m_output_var = toupper(value);
      handled = true;
    }
    else if(param == "switching_var") {
      m_switching_var = toupper(value);
      handled = true;
    }
    else if(param == "filename") {
      handled = true;
      m_gt_grid_filename = value;
    }
    else if(param == "mirror_grid") {
      handled = isBoolean(value);
      m_mirror_grid = (tolower(value) == "true");
    }
    else if(param == "mode") {
      handled = true;
      m_sim_run = (tolower(value) == "sim");
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }


  // Get Latitude Origin from .MOOS Mission File
  bool latOK = m_MissionReader.GetValue("LatOrigin", m_lat_origin);
  if(!latOK) {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return(false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", m_long_origin);
  if(!lonOK){
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return(false);
  }
  
  // Load the ground truth file;
  if (m_sim_run) {
    if (m_gt_grid_filename != "") {
      populateFromFile(m_lat_origin,m_long_origin);
      
    } else  {
    reportConfigWarning("No ground truth filename found in config block");
    }
  }

  Notify("GRID_READY", "true");
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void GridSwitcher::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  
  std::set<std::string>::iterator itr;
  for(itr = m_input_vars.begin(); itr != m_input_vars.end(); itr++) {
    if (*itr != "")
      Register(*itr, 0);
  }
  for(itr = m_input_vars_delta.begin(); itr != m_input_vars_delta.end(); itr++) {
    if (*itr != "")
      Register(*itr, 0);
  }
   
  Register(m_switching_var, 0);
  Register("ROUTE_FOUND", 0);
  Register("ROUTE_FOUND_OTHER", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool GridSwitcher::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: pGridSwitcher                         " << endl;
  m_msgs << "============================================" << endl;


  m_msgs << "Current Grid Label = " << m_current_grid_label << endl;;
  if (m_grids_received.find("psg") != m_grids_received.end() ) {
    m_msgs << " + Recieved Search Grid" << endl;
  }
  if (m_grids_received.find("cons") != m_grids_received.end() ) {
    m_msgs << " + Recieved Consensus Grid" << endl;
  }
  if (m_grids_received.find("gpr") != m_grids_received.end() ) {
    m_msgs << " + Recieved GPR Grid" << endl;
  }
  if (m_grids_received.find("gt") != m_grids_received.end() ) {
    m_msgs << " + Recieved Ground Truth Grid" << endl;
  }
  if (m_grids_received.find("cons_var") != m_grids_received.end() ) {
    m_msgs << " + Recieved Consensus Variance Grid" << endl;
  }
  if (m_grids_received.find("obs") != m_grids_received.end() ) {
    m_msgs << " + Recieved Obstacle Grid" << endl;
  }
  
  return(true);
}

  
// ---------------------------------------------------------
// -replaceGridLabel
//   Example change pts={.....}....,label=cons
//               to pts={.....}....,label=ida

std::string GridSwitcher::replaceGridLabel(std::string grid_spec, std::string vname)
{
  std::vector<std::string> svector = parseString(grid_spec, '=');
  unsigned int length = svector.size();
  if (length <1){
    reportRunWarning("Not able to replace label");
    return(grid_spec);
  }
  std::string old_label = svector[length-1];
  // replace the last entry which should be the new label
  svector[length-1] = vname;
  
  std::string relabeled_spec = "";
  for (unsigned int i=0; i < length; i++) {
    relabeled_spec += svector[i];
    if (i < (length-1)) {
	relabeled_spec += "=";
    }
  }
    
  return(relabeled_spec);
}


// ---------------------------------------------------------
// -replaceDeltaGridLabel
//      Example change cons@0,depth,10:.....
//                 to   ida@0,depth,10:.....

std::string GridSwitcher::replaceDeltaGridLabel(std::string grid_spec, std::string vname)
{
  std::vector<std::string> svector = parseString(grid_spec, '@');
  unsigned int length = svector.size();
  if (length <1){
    reportRunWarning("Not able to replace label");
    return(grid_spec);
  }
  std::string old_label = svector[0];
  // replace the first entry which should be the new label
  svector[0] = vname;
  
  std::string relabeled_spec = "";
  for (unsigned int i=0; i < length; i++) {
    relabeled_spec += svector[i];
    if (i < (length-1)) {
	relabeled_spec += "@";
    }
  }
    
  return(relabeled_spec);
}



//------------------------------------------------------------
// Procedure: populateFromFile()
//
// puts grid from file into m_filegrid
// Directly copied from iPingSim.  Should be put into a
// standalone class
//
bool GridSwitcher::populateFromFile(double lat, double lon) 
{
  
  // open file
  ifstream newfile;

  newfile.open(m_gt_grid_filename,ios::in);
  
  if (newfile.is_open()) {

    // read in file to string
    string poly_spec;
    getline(newfile, poly_spec); // discard first line
    getline(newfile, poly_spec); // get second line    
    newfile.close();

    // parse string and convert
    // get cell size string
    int cell_size_end = poly_spec.find(" # ");
    string cell_size_str = poly_spec.substr(0, cell_size_end);
    poly_spec = poly_spec.substr(cell_size_end+3);
  
    // get border string
    int border_start = poly_spec.find("{");
    int border_end = poly_spec.find(" # ");
    string border_str = poly_spec.substr(border_start, border_end-border_start);

    string convex_spec = "pts=" + border_str + "," + cell_size_str + ",";
    
    convex_spec += "cell_vars=depth:0:var:0,cell_min=depth:0,cell_max=depth:30,cell_min=var:0.0001,cell_max=var:100,";

    int cell_start = poly_spec.find("@");

    string cell_data = poly_spec.substr(cell_start+1);


    // reformat cell data
    size_t pos = 0;
    string token;
    int colon_pos;
    
    while ((pos = cell_data.find(',')) != std::string::npos) {
      
      token = cell_data.substr(0, pos);
      colon_pos = token.find(":");
      convex_spec += token.substr(0,colon_pos) + ":depth" + token.substr(colon_pos) + "var:100,";     
      cell_data.erase(0, pos+1);
      
    }

    colon_pos = token.find(":");
    convex_spec += cell_data.substr(0,colon_pos) + ":depth" + cell_data.substr(colon_pos) + "var:100";     
    XYConvexGrid temp_grid = string2ConvexGrid(convex_spec);

    if(temp_grid.size() == 0)
      reportConfigWarning("Unsuccessful ConvexGrid construction.");
     
    temp_grid.set_label("gt");
        
    m_gt_grid_from_file = EsriBathyGrid(temp_grid,lat,lon,0);

    return true;
  }

  reportConfigWarning("Ground truth grid file not found.");
  
  return false;

}


bool GridSwitcher::recalcGtGrid(std::string grid_spec) {
  
  XYConvexGrid temp_grid = string2ConvexGrid(grid_spec);
  temp_grid.set_label("gt");

  // don't care about lat or long, just x and y
  EsriBathyGrid temp_esri(temp_grid, m_lat_origin, m_long_origin, 0);

  // setup the mirror line
  XYSegList border_sl = temp_esri.getBorderSegList();
  m_x1 = 0.5 * ( border_sl.get_vx(0) + border_sl.get_vx(3) );
  m_y1 = 0.5 * ( border_sl.get_vy(0) + border_sl.get_vy(3) );
  m_x2 = 0.5 * ( border_sl.get_vx(1) + border_sl.get_vx(2) );
  m_y2 = 0.5 * ( border_sl.get_vy(1) + border_sl.get_vy(2) );
  
  // update the gt_grid so that it is similar in size
  // to the one in this spec
  unsigned int idx, size = temp_esri.size();
  for (idx = 0; idx<size; idx++) {
    // get the x and y values for this cell
    double x = 0.0;
    double y = 0.0;
    double depth = 0.0;
    double var = 0.0;
    
    temp_esri.getCellData(idx, x, y, depth, var);

    // Find the gt values in the other gt grid from the file
    double depth_gt = 0.0;
    double var_gt = 0.0;
    double full_grid_x = 0.0;
    double full_grid_y = 0.0;
    if (m_mirror_grid) {
      // mirror the nav_x and nav_y about the mirror line
      mirrorPoint(x, y, full_grid_x, full_grid_y);
    } else {
      full_grid_x = x;
      full_grid_y = y;
    }
    m_gt_grid_from_file.getCellDataXY(full_grid_x, full_grid_y, depth_gt, var_gt);

    // replace the values in this cell
    std::vector<std::string> cell_vars;
    cell_vars.push_back("depth");
    cell_vars.push_back("var");
    
    std::vector<double> cell_vals;
    cell_vals.push_back(depth_gt);
    cell_vals.push_back(var_gt);
    
    temp_esri.updateCellValueIDX(idx, cell_vars, cell_vals);
  }

  // Now update the m_gt_grid
  //m_gt_grid = temp_esri;
 
  // and update the spec in the map
  m_grids_received["gt"] = temp_esri;
  m_need_to_update_full_grid["gt"] = false;
  m_need_to_update_grid_delta["gt"] = false;

  string gt_depth_str = temp_esri.getDepthVectorString();
  Notify("GROUND_TRUTH_DEPTH", gt_depth_str);
  
  return(true);
}


//------------------------------------------------------
// Procedure handleConfigInputVars()
//  Examples : VIEW_GRID_CONS_LOCAL, VIEW_GRID_GPR_LOCAL, VIEW_GRID_PSG_LOCAL

bool GridSwitcher::handleConfigInputVars(std::string val)
{
   bool all_ok = true;
  
  vector<string> msgs = parseString(val, ',');
  for(unsigned int i=0; i<msgs.size(); i++) {
    string msg = stripBlankEnds(msgs[i]);
    // Check if non-zero length
    if(msg.length() > 0) {
      m_input_vars.insert(msg);
      m_input_vars_delta.insert( (msg + "_DELTA") );
    } else {
      all_ok = false;
    }
  }
  return(all_ok);
}


//---------------------------------------------------------
// Procedure handleFullGridMsg
bool GridSwitcher::handleFullGridMsg(std::string sval)
{
  // This is one of the input grids - the entire
  // grid
  std::string grid_label = getGridLabel(sval);
  if (grid_label == "")
    return(false);
  
  XYConvexGrid temp_grid = string2ConvexGrid(sval);
  
  // update the grid if it already exists in the map,
  // otherwise add it and set the flags.
  // If we have recieved a new grid, we don't need to
  // send the deltas, since those are now irrelevant. 
  if (m_grids_received.find(grid_label) != m_grids_received.end()) {
    m_grids_received.at(grid_label).setXYConvexGrid(temp_grid);
    
    m_need_to_update_full_grid.at(grid_label) = true;
    m_need_to_update_grid_delta.at(grid_label) = false;
  } else {
    EsriBathyGrid temp_esri(temp_grid, m_lat_origin, m_long_origin, 0);
    m_grids_received[grid_label] = temp_esri;
    m_need_to_update_full_grid[grid_label] = true;
    m_need_to_update_grid_delta[grid_label] = false;
  }
  
  // recalc gt grid if needed
  if (m_recalc_gt_grid and ( (grid_label == "cons") or (grid_label == "gpr") ) ) {
    recalcGtGrid(sval);
    m_recalc_gt_grid = false;
  }
  return(true);
}



//---------------------------------------------------------
// Procedure handleDeltaGridMsg
//     Ex. "label @ index,cix0,delta0,cix1,delta1:index,..."
bool GridSwitcher::handleDeltaGridMsg(std::string sval)
{
  // This is one of the input delta grids
  XYGridUpdate update = stringToGridUpdate(sval);
  if(not update.valid())
    return(false);
  
  std::string delta_grid_label = update.getGridName();
  
  // Check if we already have the full grid for this delta
  if (m_grids_received.find(delta_grid_label) != m_grids_received.end()) {
    bool ok = m_grids_received.at(delta_grid_label).processGridDelta(sval);
    if (not ok)
      return(false);
    
    m_need_to_update_grid_delta.at(delta_grid_label) = true;
    
  } else {
    return(false);
  }
  return(true);
}


//  Procedure:  mirrorPoint(p1x, p1y, &p2x, &p2y).
//              mirrors point p1 about the line
//              defined by m_x1, m_y1 to m_x2, m_y2
//              returns p2x and p2y by reference 
void GridSwitcher::mirrorPoint(double p1x, double p1y, double &p2x, double &p2y) {
  double dx = m_x2 - m_x1;
  double dy = m_y2 - m_y1;

  double a = (dx*dx - dy*dy)  / (dx*dx + dy*dy);
  double b = 2*dx*dy / ( dx*dx + dy*dy );

  p2x = a*(p1x - m_x1) + b*(p1y - m_y1) + m_x1;
  p2y = b*(p1x - m_x1) - a*(p1y - m_y1) + m_y1;
  
  return;
}



