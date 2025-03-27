/************************************************************/
/*    NAME: Nick Gershfeld                                  */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BathyGrider.cpp                                 */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <cmath>
#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "BathyGrider.h"
#include "XYFormatUtilsConvexGrid.h"
#include "NodeMessage.h"

using namespace std;

//---------------------------------------------------------
// Constructor

BathyGrider::BathyGrider()
{
  m_new_time = MOOSTime();
  m_gpr_time = MOOSTime();
  m_cons_time = MOOSTime();

}

//---------------------------------------------------------
// Destructor

BathyGrider::~BathyGrider()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool BathyGrider::OnNewMail(MOOSMSG_LIST &NewMail)
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


    if (key == "NAV_X") { 
      m_x = msg.GetDouble();
      m_x_time = mtime;      
    } else if (key == "NAV_Y") {
      m_y = msg.GetDouble();
      m_y_time = mtime;
    } else if (key == "PING_DISTANCE") {
      m_dist = msg.GetDouble();
      m_dist_time = mtime;
    } else if (key == "PING_CONFIDENCE") {
      m_conf = msg.GetDouble();
      m_conf_time = mtime;
    } else if (key == "SURVEYING") {
      m_surveying = (sval == "true");
    } else if (key == "EXPLORING") {
      m_exploring = (sval == "true");
    } else if (key == "TRANSITING") {
      m_transiting = (sval == "true");
    } else if (key == "CONSENSUS_DATA") {
      
      vector<double> msg_estimate;
      vector<double> msg_variance;
      double msg_deg_connectedness;
      string vname;
      bool ok;
      
      ok = handleConsensusSpec(sval, vname, msg_estimate, msg_variance,
			       msg_deg_connectedness);
      if (ok) {
	m_consensus.receiveConsEstMessage(vname, mtime, msg_estimate, msg_variance,
					  msg_deg_connectedness);
	m_total_other_submissions ++;
      } else {
	reportRunWarning("Incoming Consensus Data Incomplete");
      }
      
    } else if (key == "NEW_CONSENSUS_REQUESTED") {

      if (m_consensus.receiveRequestMessage(dval,MOOSTime())) {
	m_consensus.markNewConsensusRequested( MOOSTime() );
	m_total_requests_recieved ++;
	this->sendOutOwnEstimate();
      }
      
    } else if (key != "APPCAST_REQ") { // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
    
  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool BathyGrider::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool BathyGrider::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Bail right away if not surveying
  if (!(m_surveying or m_exploring or m_transiting)) {
    m_gpr_time = MOOSTime();
    m_cons_time = MOOSTime();
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  /////////////////////////////////////////////////////////
  //   GPR Estimation code section
  /////////////////////////////////////////////////////////

  // There are three steps:
  // 1. Check if we have received a new valid set of observations,
  //    and if so, then add it to the GPR object.
  //
  // 2. Perform one step of the estimation during the specified
  //    interval. Only compute one step every m_appticks_to_skip
  //    appticks
  //
  // 3. If we are finished with all steps, then submit the estimate
  //    to the consensus, publish, and clean up for the next round
  
  // GPR Step 1. 
  // Is there a new set of valid observations?
  // All these things must be true:
  bool update_new = ( (MOOSTime() - m_new_time) >= m_time_between_estimates );
  bool x_new = ( (MOOSTime() - m_x_time) <= m_time_between_estimates );
  bool y_new = ( (MOOSTime() - m_y_time) <= m_time_between_estimates );
  bool dist_new = ( (MOOSTime() - m_dist_time) <= m_time_between_estimates );
  bool conf_new = ( (MOOSTime() - m_conf_time) <= m_time_between_estimates );
  bool high_conf = (m_conf > 99);
  
  // check if new observation update
  if (update_new && x_new && y_new && dist_new && conf_new && high_conf) {
    
    // convert and add to grid
    double converted_depth = m_dist*m_conversion_factor;
    vector<double> cell_vals;
    cell_vals.push_back(converted_depth);
    cell_vals.push_back(m_conf);
 
    vector<double> state; 
    state.push_back(m_x);
    state.push_back(m_y);
    
    m_gpr.recordObservation(state,converted_depth);

    // update observation count
    m_count++;
    m_new_time = MOOSTime();
 
  }

  
  // GPR Step 2. 
  // Perform one GPR iteration every few Appticks
  if (m_apptick_counter < m_appticks_to_skip) { // keep skipping
    m_apptick_counter++;
    
  } else { // skipped enough, run iteration

    // reset counter
    m_apptick_counter = 0;
    
    // If we are done with this set of iterations,
    // get a new sample size and number of iterations.
    if (m_iterations_completed == 0) {
      
      // Calculate the sample size and iterations
      m_gpr.calcSampleSizeAndIterations();
      
      // Get number of iterations
      m_iterations_to_do = m_gpr.getFastGPRIterations();
    }
    
    
    // During each GPR iteration     
    
    // Sample the observations and build the covariance matrix
    // using in the omit list and distance threshold
    m_gpr.sampleAndBuild(m_omit_list, m_omit_dist_thresh);
    
    // update the cell for each x,y grid location based on index
    for (unsigned int idx = 0; idx < m_grid_gpr.size(); idx++) {

      // get cell info
      double cell_x, cell_y, cell_depth, cell_var;
      m_grid_gpr.getCellData(idx, cell_x, cell_y, cell_depth, cell_var);

      // calculate estimate at x, y
      vector<double> state;
      state.push_back(cell_x);
      state.push_back(cell_y);

      // Check that this cell is not on the omit list
      // We don't update it for speed. 
      bool in_omit_list = false;
      double epsilon = .01;
      
      for (unsigned int j=0; j<m_omit_list.size(); j++) {
	double dd = dist(state, m_omit_list[j]);
	if (dd < epsilon){
	  in_omit_list = true;
	  break;
	}
      }

      if(in_omit_list)
	continue;

      // get an estimate of val and covariance
      double val = -1;
      double covar = -1;
      m_gpr.estimateFastGPRFromSampleSet(state, val, covar);
      
      // Update running average
      // This approach handles the case with m_iterations_completed = 0
      double depth = (cell_depth*m_iterations_completed+val)/(m_iterations_completed+1);
      double variance = (cell_var*pow(m_iterations_completed,2)+covar)/(pow((m_iterations_completed+1),2));

      // and update the grid with the new running average
      vector<double> cell_vals;
      cell_vals.push_back(depth);
      cell_vals.push_back(variance);
      m_grid_gpr.updateCellValueIDX(idx,m_cell_vars,cell_vals);

      // build omit list if completed iterations and the variance is small.
      // need to subtract one from the number of iterations to do because
      // we increment the iterations completed AFTER this.
      if ((m_iterations_completed == (m_iterations_to_do-1) ) and (variance <= m_variance_threshold)) {
	  m_omit_list.push_back(state);
	  
	  // send a circle to mark the grid
	  XYCircle xyCircle_Finished(state[0], state[1], 3.0);
	  xyCircle_Finished.set_color("edge","limegreen");
	  xyCircle_Finished.set_label(std::to_string(m_omit_list.size()));
	  xyCircle_Finished.set_edge_size(3.0);
	  //Notify("VIEW_CIRCLE", xyCircle_Finished.get_spec());
		 
      }
    }

    // increment the number of iterations. 
    m_iterations_completed++;


    // GPR Step 3.
    // If we have completed all the iterations to do, then
    // submit it to the consensus, and publish.  
    // 
    if (m_iterations_completed == m_iterations_to_do) {

      // Submit to the consensus
      vector<double> depth_vector = m_grid_gpr.getDepthVector();
      vector<double> variance_vector = m_grid_gpr.getVarianceVector();
      m_consensus.submitEstimate(MOOSTime(),depth_vector,variance_vector);
      m_total_own_submissions ++;

      // Reset the time
      m_gpr_time = MOOSTime();
      
      // Now send the deltas and update the old grid
      bool found_delta = false;
      string delta_spec_gpr = m_grid_gpr.getDeltaSpec(m_delta_thresh, found_delta);
      if (found_delta) {
	Notify("VIEW_GRID_GPR_LOCAL_DELTA", delta_spec_gpr);
	m_grid_gpr.setOldGridToNew();  // no more delta, we have sent all the updates
      }
      m_iterations_completed = 0;
      
    }
  } // end of skipping for GPR
  

  /////////////////////////////////////////////////////////
  //   Kalman Consensus code section
  /////////////////////////////////////////////////////////

  // Kalman Consensus Step 1.
  // Prompt a new consensus if enough time has elapsed
  // since the last consensus was completed AND we have
  // waited long enough for responses. 
  bool cond1 = (MOOSTime() - m_cons_time) >= m_cons_period;
  bool cond2 = (MOOSTime() - m_consensus.getNewConsensusRequestTime() ) >= ( static_cast<double>(m_max_iterations) * m_cons_waittime + 1.0);
  if (cond1 and cond2) {
    
    NodeMessage node_message;
    node_message.setSourceNode(m_vname);
    node_message.setVarName("NEW_CONSENSUS_REQUESTED");
    node_message.setDoubleVal(MOOSTime());
    node_message.setColor("white");
    node_message.setSourceApp("pBathyGrider");
    node_message.setDestNode("all");
    string msg = node_message.getSpec();
    
    Notify("NODE_MESSAGE_LOCAL", msg);

    m_consensus.markNewConsensusRequested( MOOSTime() );
    m_total_requests_initiated ++;

    // and also send out own estimate!
    this->sendOutOwnEstimate();
   
  }

  //  Kalman Consensus Step 2.
  //  Perform the consensus if requested and everything is ready
  bool new_cons_requested = m_consensus.newConsensusRequested();
  
  if (new_cons_requested) {
    
    if (m_consensus.readyToPerformConsensus(MOOSTime(), m_max_iterations )) {
      
      vector<double> consensus_val;
      vector<double> consensus_var;
      bool use_running_consensus_when_neighbor_est_available = false;
      bool use_running_consensus_if_no_neighbor_est_available = true;
      
      bool ok_consensus;
      double cons_timer = MOOSTime();
      ok_consensus = m_consensus.newConsensusItr(MOOSTime(),
						 m_multivariate_states,
						 use_running_consensus_when_neighbor_est_available,
						 use_running_consensus_if_no_neighbor_est_available,
						 consensus_val,
						 consensus_var);
      if(ok_consensus) {

	for (int i=0; i<m_grid_cons.size(); i++) {
	  vector<double> cell_vals;
	  cell_vals.push_back(consensus_val[i]);
	  cell_vals.push_back(consensus_var[i]);
	  m_grid_cons.updateCellValueIDX(i,m_cell_vars,cell_vals);
	}
	// Send out the delta and update
	bool found_delta = false;
	string delta_spec_gpr = m_grid_cons.getDeltaSpec(m_delta_thresh, found_delta);
	if (found_delta) {
	  Notify("VIEW_GRID_CONS_LOCAL_DELTA", delta_spec_gpr);
	  m_grid_cons.setOldGridToNew();  // no more delta, we have sent all the updates
	}
	// We will handle the variance spec in pGridSwitcher..
       	this->sendOutOwnEstimate();
	m_cons_time = MOOSTime();
	m_cons_calc_timer = MOOSTime() - cons_timer;

      } else {
	reportRunWarning("Error Consensus did not complete");
      }
         
    }  // end of if ready to perform consensus
    
  } // end of if new consensus requested
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool BathyGrider::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  
  string grid_config;

  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());
  
  else if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    
    list<string>::reverse_iterator p;
    for(p=sParams.rbegin(); p!=sParams.rend(); p++) {
      string orig = *p;
      string config_line = *p;
      string param = toupper(biteStringX(config_line, '='));
      string value = config_line;

      if (param == "GRID_CONFIG") {
	unsigned int len = grid_config.length();
	if((len > 0) && (grid_config.at(len-1) != ','))
	  grid_config += ",";
	grid_config += value;
      }

      else if (param == "CONVERSION_FACTOR") {
	m_conversion_factor = stod(value);
      }

      else if (param == "TIME_BETWEEN_ESTIMATES") {
	m_time_between_estimates = stod(value);
      }

      else if (param == "NO_DATA_VALUE") {
	m_no_data_value = stod(value);
      }

      else if (param == "SENSOR_VARIANCE") {
	m_sensor_variance = stod(value);
      }

      else if (param == "VARIANCE_THRESHOLD") {
	m_variance_threshold = stod(value);
      }

      else if (param == "KERNEL_LENGTH_SCALE") {
	m_kernel_length_scale = stod(value);
	// also set the consensus.
	m_consensus.setKernelLengthScale(m_kernel_length_scale);
      }
      
      else if (param == "OMIT_LIST_DIST_THRESH") {
	m_omit_dist_thresh = stod(value);
      }

      else if (param == "APPTICKS_TO_SKIP") {
	m_appticks_to_skip = stoi(value);
      }

      else if (param == "CONSENSUS_TIMEOUT") {
	m_cons_timeout = stod(value);
      }

      else if (param == "CONSENSUS_WAIT_TIME") {
	m_cons_waittime = stod(value);
      }

      else if (param == "CONSENSUS_PERIOD") {
	m_cons_period = stod(value);
      }

      else if (param == "KALMAN_PROCESS_NOISE") {
	m_kalman_process_noise = stod(value);
      }
      else if (param == "MAX_ITERATIONS") {
	m_max_iterations = stod(value);
      }

      else if (param == "DELTA_GRID_UPDATE_THRESH") {
	m_delta_thresh = stod(value);
      }

      else {
        reportUnhandledConfigWarning(orig);
      }
	
      
    }
  }

  double LatOrigin = 0.0;
  double LonOrigin = 0.0;
  
  // Get Latitude Origin from .MOOS Mission File
  bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
  if(!latOK) {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return(false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
  if(!lonOK){
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return(false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool vnameOK = m_MissionReader.GetValue("Community", m_vname);
  if(!lonOK){
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return(false);
  }

  // initialize grids
  XYConvexGrid temp_grid = string2ConvexGrid(grid_config);

  if(temp_grid.size() == 0)
    reportConfigWarning("Unsuccessful ConvexGrid construction.");

  temp_grid.set_label("gpr");
  m_grid_gpr = EsriBathyGrid(temp_grid,LatOrigin,LonOrigin,m_no_data_value);
  Notify("VIEW_GRID_GPR_LOCAL", m_grid_gpr.getGridSpec() );
  
  temp_grid.set_label("cons");
  m_grid_cons = EsriBathyGrid(temp_grid,LatOrigin,LonOrigin,m_no_data_value);
  Notify("VIEW_GRID_CONS_LOCAL", m_grid_cons.getGridSpec() );

  string xy_str = m_grid_cons.getXYString();
  Notify("GRIDCELL_XY", xy_str);

  // Save time later by completing some bookkeeping
  // Init multivariate_states based on grid
  for (int idx = 0; idx < m_grid_gpr.size(); idx++) {
      
    double cell_x, cell_y, cell_depth, cell_var;
    std::vector<double> multivariate_state;
    
    m_grid_gpr.getCellData(idx, cell_x, cell_y, cell_depth, cell_var);
    multivariate_state.push_back(cell_x);
    multivariate_state.push_back(cell_y);
    m_multivariate_states.push_back(multivariate_state); 
  }
  
  // init m_cell_vars
  m_cell_vars.push_back("depth");
  m_cell_vars.push_back("var");

  
  // Init gpr and consensus
  m_gpr = SimpleGPR(1,m_sensor_variance, m_kernel_length_scale);

  m_consensus = SimpleKalmanConsensus(m_cons_timeout, m_cons_waittime);
  m_consensus.setVName(m_vname);
  m_consensus.setProcessNoiseVariance(m_kalman_process_noise);

  registerVariables();
  
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void BathyGrider::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("PING_DISTANCE", 0);
  Register("PING_CONFIDENCE", 0);
  Register("SURVEYING", 0);
  Register("EXPLORING", 0);
  Register("TRANSITING", 0);
  Register("NEW_CONSENSUS_REQUESTED", 0);
  Register("CONSENSUS_DATA", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool BathyGrider::buildReport() 
{
  m_msgs << "surveying: " << m_surveying << endl;
  //m_msgs << m_grid_gpr.getGridSpec() << endl;
  m_msgs << "Observation count: " << m_count << endl;
  m_msgs << "last observed time: " << MOOSTime() - m_new_time << endl;
  m_msgs << "x time: " << MOOSTime() - m_x_time << endl;
  m_msgs << "y time: " << MOOSTime() - m_y_time << endl;
  m_msgs << "dist time: " << MOOSTime() - m_dist_time << endl;
  m_msgs << "conf time: " << MOOSTime() - m_conf_time << endl;
  m_msgs << " ------------ GPR -------------------------------------------- " << endl;
  m_msgs << "last gpr time: " << MOOSTime() - m_gpr_time << endl;
  m_msgs << "    " << m_iterations_completed << " of " << m_iterations_to_do << " iterations to do." << endl;
  m_msgs << "    Number of cells to omit = " << m_omit_list.size() << endl;
  m_msgs << " -------------Consensus---------------------------------------  " << endl;
  m_msgs << "  Last consensus was performed " << MOOSTime() - m_cons_time << " seconds ago" << endl;
  m_msgs << "  Last consensus was requested " << MOOSTime() - m_consensus.getNewConsensusRequestTime() << " seconds ago"<< endl;
  m_msgs << "  Last consensus took " << m_cons_calc_timer << " seconds" << endl;
  m_msgs << "New Consensus requested = " <<  m_consensus.newConsensusRequested() << endl;
  m_msgs << "    Total own requests initiated  = " << m_total_requests_initiated << endl;
  m_msgs << "    Total other requests recieved = " << m_total_requests_recieved << endl;
  m_msgs << "    Total own submissions   = " << m_total_own_submissions << endl;
  m_msgs << "    Total other submissions = " << m_total_other_submissions << endl;
  std::string  con_spec = m_consensus.replyToRequestSpec();
  m_msgs << "    Last consensus spec length = " << con_spec.size() << endl;
  m_msgs << "    Number of cells = " << m_grid_gpr.size() << endl;

  return(true);
}


//-------------------------------------------------------
//  Handle an incoming consensus Spec
//      basic parsing.
//      Format is
//      vname=abe;;; if no own estimate and variance is present
//      vname=abe;1,2,3,4;5,6,7,8;2 if own estimate and variance
//      is present, and number of connected agents is known
//      Returns false message is not completely correct,
//      but otherwise will attempt to read as much as possible
bool BathyGrider::handleConsensusSpec(string input, string &vname,
				      vector<double> &estimate,
				      vector<double> &variance,
				      double &deg_connectedness)
{
  
  vector<string> svector = parseString(input, ';');
  // will allways be at least one entry
  if ( biteStringX(svector[0], '=') == "vname") 
    vname = svector[0];
  else
    return(false);

  // check length before accessing
  if (svector.size() < 2)
    return(false);
  
  if (svector[1] == "") {
    return(false);
    
  } else {
    std::vector<string> estimate_vstring = parseString(svector[1], ',');
    for(unsigned int i=0; i<estimate_vstring.size(); i++) {
      
      double temp_val = 0.0;
      bool ok = setDoubleOnString( temp_val, estimate_vstring[i]);
      if (ok) {
	estimate.push_back( temp_val );
      } else {
	return(false);
      }
    }
  }
  
  // check length before accessing
  if (svector.size() < 3) 
    return(false);
  
  if (svector[2] == "") {
    return(false);
    
  } else {
    std::vector<string> variance_vstring = parseString(svector[2], ',');
    for(unsigned int i=0; i<variance_vstring.size(); i++) {
      
      double temp_val2 = 0.0;
      bool ok2 = setDoubleOnString( temp_val2, variance_vstring[i]);
      if (ok2) {
	variance.push_back( temp_val2 );
      } else {
	return(false);
      }
    }
  }

  // check length before accessing
  if (svector.size() < 4) 
    return(false);

  if (svector[3] == "") {
    return(false);

  } else {
    double temp_val3 = 0.0;
    bool ok3 = setDoubleOnString( temp_val3, svector[3]);
    if (ok3) {
      deg_connectedness = temp_val3;
    } else {
      return(false);
    }
  }
  return(true);   
}


//-----------------------------------------------------
//  Send out own estimate
void BathyGrider::sendOutOwnEstimate() {
  string reply_msg = m_consensus.replyToRequestSpec();
  
  NodeMessage node_message;
  node_message.setSourceNode(m_vname);
  node_message.setVarName("CONSENSUS_DATA");
  node_message.setStringVal(reply_msg);
  node_message.setColor("red");
  node_message.setSourceApp("pBathyGrider");
  node_message.setDestNode("all");
 
  string msg = node_message.getSpec(); 
  Notify("NODE_MESSAGE_LOCAL", msg);
}


//-------------------------------------------------
// 
double BathyGrider::dist(std::vector<double> v1, std::vector<double> v2)
  {
  if (v1.size() != v2.size())
    return(0.0);
  
  double val = 0.0;
  for (int i=0; i<v1.size(); i++){
    val = std::pow( (v1[i] - v2[i]), 2) + val;
  }
  return(std::sqrt(val));

}
