/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GroupComboAlloc.cpp                                  */
/*    DATE: July 2024                                       */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "GroupComboAlloc.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

GroupComboAlloc::GroupComboAlloc()
{
  m_nav_x = 0.0;
  m_nav_y = 0.0;
  m_nav_hdg = 0.0;
  m_vname = "";
  m_curr_option = "";
  m_active_option_name = "INTERCEPT";

  m_valid_option_lists.insert("OUTSIDE_ZONE_LIST");
  m_valid_option_lists.insert("INSIDE_ZONE_LIST");
  m_valid_option_lists.insert("INTERCEPT_LIST");
  m_valid_option_lists.insert("EXPLORE_LIST");
  m_valid_option_lists.insert("EXPLOIT_LIST");

  m_hvu_protect = false;
  m_post_behavior_updates = false;
  
  m_hvu_name = "";
  m_intercept_heading_penalty = 0.1;

  m_own_target = "";
  m_own_target_priority = 42;

  m_assigned_cost = 40.0;
  m_unassigned_penalty = 100;

  m_group_trail_range_offset = 10;
  m_primary_trail_range = 20;
  m_group_trail_angle_offset = 20;

  m_calc_marginal_cost_improvement = false;
  
}

//---------------------------------------------------------
// Destructor

GroupComboAlloc::~GroupComboAlloc()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GroupComboAlloc::OnNewMail(MOOSMSG_LIST &NewMail)
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

    } else if (key == "CONTACTS_LIST") {
      if ( !handleContactsList(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);

    } else if (key == "OPTION"){
      if (msg.GetString() != ""){
	m_curr_option = toupper(msg.GetString());
      } else
	reportRunWarning("Unhandled Mail: " + key);
    
    } else if ( m_valid_option_lists.count(key) > 0) {
      if ( !handlePopStateList(key, msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "SAMPLE_LOC") {
      if ( !handleSampleLoc(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "INTERCEPT_ESTIMATE") {
      if ( !handleInterceptEstimate(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if (key == "EXTRA_COST")  {
      if ( !handleExtraAgentCost(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);
      
    } else if ((key == "INTRUDER_LIST") || (key == "SAMPLE_LIST")) {
      m_intruder_set.clear();
      std::vector<std::string> intruder_list_vec = parseString(msg.GetString(), ',');
      for (unsigned int i = 0; i < intruder_list_vec.size(); i++){
	m_intruder_set.insert(intruder_list_vec[i]);
      }
      Notify("INTRUDER_SET", stringSetToString(m_intruder_set));
    }
    
  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GroupComboAlloc::OnConnectToServer()
{
   //registerVariables();  // commented out to reduce redundancy
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GroupComboAlloc::Iterate()
{
  AppCastingMOOSApp::Iterate();


  if (m_intruder_set.size() > 0){

    // Are we participating?
    bool participating = (m_curr_option == m_active_option_name);

    // determine best target if we are participating,
    // or, if we want to calculate the marginal cost improvement
    if (participating || m_calc_marginal_cost_improvement ){
      
      // calculate best target to intercept including own participation
      determineBestTargetToIntercept(true);
    
      // update behaviors
      if ((m_post_behavior_updates) && (participating))
	updateBehaviors();
      
    }
   
    // Also calculate cost as if not participating
    if (m_calc_marginal_cost_improvement ){
      
      // calculate best target to intercept WITHOUT including own participation
      determineBestTargetToIntercept(false);
    }
    
  } else {
    m_own_target = "";
    m_own_target_priority = 42;
    Notify("COST_WITHOUT_PARTICIPATION", 0.0);
    Notify("COST_WITH_PARTICIPATION", 0.0);
    

  }
  


  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GroupComboAlloc::OnStartUp()
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
    if(param == "hvu_name") {
      if (value != ""){
	m_hvu_name = value; 
	handled = true;
      }
      
    } else if(param == "active_option_name") {
      if (value != ""){
	m_active_option_name = toupper(value); 
	handled = true;
      }
     
    } else if (param == "intercept_heading_penalty"){
      handled = setPosDoubleOnString(m_intercept_heading_penalty, value);
    } else if (param == "primary_trail_range"){
      handled = setPosDoubleOnString(m_primary_trail_range, value);
    } else if (param == "group_trail_range_offset"){
      handled = setDoubleOnString(m_group_trail_range_offset, value);
    } else if (param == "group_trail_angle_offset"){
      handled = setDoubleOnString(m_group_trail_angle_offset, value);
    } else if (param == "post_behavior_updates") {
      handled = setBooleanOnString(m_post_behavior_updates, value);
    } else if (param == "calc_marginal_cost_improvement"){
      handled = setBooleanOnString(m_calc_marginal_cost_improvement, value);
    } else if (param == "assigned_cost"){
      handled = setPosDoubleOnString(m_assigned_cost, value);
    } else if (param == "unassigned_penalty"){
      handled = setPosDoubleOnString(m_unassigned_penalty, value);
    }
 
    
    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  // The vehicle name is the host community
  m_vname = m_host_community;

  // Initialize the pop state map with an empty set
  std::set<std::string> new_active_set;
  m_pop_state_map[m_active_option_name] = new_active_set;
 

  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void GroupComboAlloc::registerVariables()
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
  Register("INTERCEPT_ESTIMATE", 0);
  Register("SAMPLE_LOC",0);
  Register("EXTRA_COST",0);
  Register("INTRUDER_LIST", 0);
  Register("SAMPLE_LIST",0);


  // Register for the set option lists
  std::set<std::string>::iterator it;
  for (it = m_valid_option_lists.begin(); it != m_valid_option_lists.end(); it++){
    Register(*it, 0);
  }
}


//-----------------------------------------------------
// handleNodeReport(std::string msg)
//
bool GroupComboAlloc::handleNodeReport(std::string msg)
{
 
  // process incoming node record 
  NodeRecord newNodeRecord;
  newNodeRecord = string2NodeRecord(msg);
  if (!newNodeRecord.valid())
    return(false);

  // Only keep contacts that are alive:
  if ( m_contacts.count(newNodeRecord.getName()) || (newNodeRecord.getName() == m_vname) )
    m_node_rec_map[newNodeRecord.getName()] = newNodeRecord;
  
  return(true);
}



//-----------------------------------------------------
// Procedure: handleContactsList(std::string msg)
//
bool GroupComboAlloc::handleContactsList(std::string msg)
{
  // parse message
  std::set<std::string> new_contact_set;
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i=0; i<svector.size(); i++) {
    new_contact_set.insert(svector[i]);
  }
  m_contacts = new_contact_set;

  // Now clear out any old node reports
  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end();){
    if ( m_contacts.count(it->first) || (m_vname == it->first) ) {
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
bool GroupComboAlloc::handlePopStateList(std::string key, std::string msg){

  // Make a new set to erase the old set.
  std::set<std::string> new_active_set;
  // parse message
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i=0; i<svector.size(); i++) {
    new_active_set.insert(svector[i]);
  }

  // Some sets could be empty!

  // remove the "_LIST" at the end
  std::string option_name = key.substr(0, key.length()-5);
  option_name = toupper(option_name);

  m_pop_state_map[option_name] = new_active_set; 


  return(true); 
}


//---------------------------------------------------------------
// handleSampleLoc

bool GroupComboAlloc::handleSampleLoc(std::string msg)
{
  // build an intercept estimate from this
  XYPoint new_sample_point=string2Point(msg);
  if (!new_sample_point.valid())
    return(false);

  std::string intercept_spec = "";
  intercept_spec += "name=" + new_sample_point.get_label();
  intercept_spec += ",x=" + doubleToStringX(new_sample_point.get_vx(),3);
  intercept_spec += ",y=" + doubleToStringX(new_sample_point.get_vy(),3);
  
  return(handleInterceptEstimate(intercept_spec));
}


//---------------------------------------------------------------
// handleExtraAgentCost
//        Accepts updates as agent:value

bool GroupComboAlloc::handleExtraAgentCost(std::string msg)
{
  // Parse the message
  vector<string> spec = parseString(msg, ':');

  if (spec.size() != 2)
    return(false);
  
  double cost = 0.0;
  bool ok = setDoubleOnString(cost, spec[1]);

  if (!ok)
    return(false);
  
  m_agent_extra_cost[spec[0]] = cost;
  
  return(true);
}


//---------------------------------------------------------------
// handleInterceptEstimate

bool GroupComboAlloc::handleInterceptEstimate(std::string msg)
{
  if (msg == "")
    return(true);

  // Do we already have an estimate for this contact?
  double intrd_x, intrd_y;
  std::string intrd_name = "";
  parseInterceptEstimate(msg, intrd_name, intrd_x, intrd_y);

  std::list<std::string>::iterator it;
  double rec_x, rec_y;
  std::string rec_name = "";
  for (it = m_intercept_estimates.begin(); it != m_intercept_estimates.end();) {
    std::string this_estimate = *it;
    parseInterceptEstimate(this_estimate, rec_name, rec_x, rec_y);

    if (rec_name == intrd_name) {
      // we have an old record, delete it
      it = m_intercept_estimates.erase(it); 
    } else {
      ++it;
    }
  }

  // Now that we have removed the old estimate if it exists, add this new one
  m_intercept_estimates.push_back(msg); 

  return(true); 

}

//------------------------------------------------------------
// Procedure: buildReport()

bool GroupComboAlloc::buildReport() 
{
  m_msgs << "                                                          " << endl;
  m_msgs << "==========================================================" << endl;
  m_msgs << "=  Intercept (or Sample) Estimates:                       " << endl;
  std::list<std::string>::iterator sit;
  for (sit = m_intercept_estimates.begin(); sit !=  m_intercept_estimates.end(); sit++){
    m_msgs << " * " << *sit << endl;
  }

  
  m_msgs << "                                                            " << endl;
  m_msgs << "===========================================================" << endl;
  m_msgs << "=  Node Reports:                                           " << endl;

  std::map<std::string, NodeRecord>::iterator it;
  for (it = m_node_rec_map.begin(); it != m_node_rec_map.end(); it++){
    m_msgs << " -> " << it->second.getSpec() << endl;
  }
   

  
  ACTable actab(4);
  actab << "Own Target | Priority | Charlie | Delta";
  actab.addHeaderLines();
  actab << m_own_target << uintToString(m_own_target_priority) << "three" << "four";
  m_msgs << actab.getFormattedString();
 

  return(true);
}


//--------------------------------------
// Procedure: parseInterceptEstimate
//            name=badguy1,x=1,y=2

void GroupComboAlloc::parseInterceptEstimate(std::string spec, std::string &name, double &x, double &y)
{
  double new_x = 0.0;
  double new_y = 0.0; 
  std::string new_name = "to-be-set";

  vector<string> svector = parseString(spec, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    if((param == "name") && (value != ""))
      new_name = value;
    else if((param == "x") && isNumber(value))
      setDoubleOnString(new_x, value);
    else if((param == "y") && isNumber(value))
      setDoubleOnString(new_y, value);
  }

  //update by reference
  name = new_name;
  x = new_x;
  y = new_y; 
  
  return;
}



//---------------------------------------
// Procedure: determineBestTargetToIntercept
void GroupComboAlloc::determineBestTargetToIntercept(bool include_own)
{

  
  // Initialize an empty set for allocated samples
  std::set<std::string> allocated_samples; 

  // now build the assignement map
  m_assignments_map.clear();

  // Clear extra costs from previous assignemt
  m_agent_already_assigned_cost.clear();

  // Total Costs for this allocation
  double total_cost = 0.0;

  // First check if the team size is 0
  int team_size = getTotalTeamSize(include_own);
  if (team_size == 0){
    // post the total cost to let all of these
    // go unallocated and return;
    total_cost = static_cast<double>(m_intruder_set.size()) * m_unassigned_penalty;
    postAssignmentsMap(include_own, total_cost);
    return;
  }

  // keep iterating until all samples have been allocated
  // The number of samples is greater than the number of
  // agents
  // The cost map is recomputed each time.
  int count = 0;
  bool found_first_own_target = false;
  bool rebuild_cost_map = true;
  while (allocated_samples.size() < m_intruder_set.size()){

    count += 1;
    if (count > 100)
      break;
    
    // Step 1.  Get the cost matrix
    // For some memory-related reason I cannot yet find, we
    // need to copy of the Dist Matrix, clear the original, and
    // and then reset it in this weird way. The majority of the
    // code here is to do this process and print out the results
    // for debugging.
    buildCostMap(include_own, rebuild_cost_map, allocated_samples);

    // Don't rebuild the cost map next time(s)
    rebuild_cost_map = false;

    std::vector<std::vector<double>> DistMatrix;
    DistMatrix = getDistMatrixFromCostMap();
  
    std::vector<std::vector<double>> copyDistMatrix = DistMatrix;
    DistMatrix.clear();
    
    std::vector<double> agent1_cost;
    if (copyDistMatrix.size() > 0){
      for (unsigned int j = 0; j < copyDistMatrix[0].size(); j++){
	agent1_cost.push_back(copyDistMatrix[0][j]);
      }
    }
    DistMatrix.push_back(agent1_cost);
    
    for (unsigned int i = 1; i < copyDistMatrix.size(); i++){ 
      DistMatrix.push_back(copyDistMatrix[i]); 
    }
    
    
    // print it for debugging
    /*
    std::cout << "       Reallocated DistMatrix at:" << &DistMatrix << std::endl; 
    for (unsigned int i = 0; i < DistMatrix.size(); i++){
      std::cout << "i = " << uintToString(i) << " Cost Vec: ";
      for (unsigned int j = 0; j < DistMatrix[i].size(); j++){
	std::cout << doubleToStringX(DistMatrix[i][j], 2) << ", ";
      }
      std::cout << " the size was " << uintToString(DistMatrix[i].size()) << std::endl;
    }
    std::cout << std::endl;
    */
    
    // Step 2. Solve it for this set of teammates 
    std::vector<int> assignment;
    double cost = HungAlgo.Solve(DistMatrix, assignment);

    total_cost = total_cost + cost; 
    
    /*
    std::cout << "  -->  Solved assignments with cost = " << doubleToStringX(cost, 4) << std::endl;
    std::cout << "  ---> Assignments: ";
    for (unsigned int i = 0; i < assignment.size(); i++){
      std::cout << "i = " << uintToString(i) << " to task " << intToString(assignment[i]) << ", ";
    }
    std::cout << std::endl;
    */
    
    // Step 3.  update the teammates that were assigned.
    for (unsigned int i = 0; i < assignment.size(); i++){
      if (assignment[i] == -1){
	// this agent was not assigned in this round
	continue; 
      }

      // this agent was assigned.  Record it

      // m_intruder_vec[ assignment[i] ] is the name of the intruder that the ith vehicle is
      //                                 assigned to
      std::string intruder_assigned_to =  m_intruder_vec[ assignment[i] ];

      // m_unalloc_teammates_vec[i]      is the name of the ith unallocated teammate   
      std::string name_of_teammate  = m_unalloc_teammates_vec[i];

      if (m_assignments_map.count( intruder_assigned_to) == 0) {
	// this is the first agent assigned, need something to push back to
	std::vector<std::string> new_assignment_vec;
	new_assignment_vec.push_back( name_of_teammate );
	m_assignments_map[ intruder_assigned_to ] = new_assignment_vec; 
	
      } else {
                       
	m_assignments_map[ intruder_assigned_to ].push_back( name_of_teammate );
      }

      // This sample/intruder was assigned, add it to the allocated set
      allocated_samples.insert(intruder_assigned_to);

      // This agent was assigned, add the extra cost
      if (m_agent_already_assigned_cost.count(name_of_teammate) == 0){
	m_agent_already_assigned_cost[name_of_teammate] =  m_assigned_cost;
      } else {
	m_agent_already_assigned_cost[name_of_teammate] = m_agent_already_assigned_cost[name_of_teammate] + m_assigned_cost;
      }

      // evesdrop on this for our own future reference;
      // only do so if we are actually participating
      // This might be better to move somewhere else?
      bool actually_participating = (m_curr_option == m_active_option_name);

      if ((name_of_teammate == m_vname) && (actually_participating) && (!found_first_own_target)) {
	m_own_target = intruder_assigned_to;
	m_own_target_priority = m_assignments_map[intruder_assigned_to].size();
	Notify("OWN_TARGET", m_own_target);
	Notify("OWN_TARGET_PRIORITY", to_string(m_own_target_priority));
	found_first_own_target = true;  // only post the first time
      }
    }
      
  } // end of while loop

  postAssignmentsMap(include_own, total_cost);

  return;

}


//---------------------------------------
// Procedure: updateBehaviors
//            This assumes we have updated
//            m_own_target and m_own_target_priority
//            using determineBestTargetToIntercept()
//
// 
void GroupComboAlloc::updateBehaviors()
{
  //sanity checks
  if (m_own_target == "")
    return;

  // First, do we have a node report for this target?
  if (m_node_rec_map.count(m_own_target) == 0){
    // we need to use another behavior such
    // as a waypoint behavior to head to the location
    Notify("SEARCH_FOR_TARGET", "true");

    // get the x and y info from the intercept message

    double intrd_x, intrd_y;
    std::string intrd_name = "to-be-set";
  
    // check the list of estimates
    bool found_estimate = false;
    std::list<std::string>::iterator lit;
    for (lit = m_intercept_estimates.begin(); lit != m_intercept_estimates.end(); lit++){
      parseInterceptEstimate(*lit, intrd_name, intrd_x, intrd_y);
      if (m_own_target == intrd_name){
	// we found a record
	// mark it and stop
	found_estimate = true;
	break;
      }
    }

    if (!found_estimate)
      return;

    XYPoint newPoint;
    newPoint.set_vx(intrd_x);
    newPoint.set_vy(intrd_y);
    Notify("SEARCH_WAYPOINT_UP", "point=" + newPoint.get_spec());

    return;
  }


  // Otherwise we need to update the trail behavior
  Notify("SEARCH_FOR_TARGET", "false");

  
  // Keep the radius the same, but adjust the angle to that the
  // trail point is between the hvu and the intruder

  double hvu_x, hvu_y, intrd_x, intrd_y;

  //sanity checks first
  if (m_node_rec_map.count(m_hvu_name) == 0)
    return;

  // we know a node record for the intruder
  // exists in the map

  hvu_x = m_node_rec_map[m_hvu_name].getX();
  hvu_y = m_node_rec_map[m_hvu_name].getY();
  intrd_x = m_node_rec_map[m_own_target].getX();
  intrd_y = m_node_rec_map[m_own_target].getY();

  double best_trail_angle = relAng(intrd_x, intrd_y, hvu_x, hvu_y);

  // and set the trail info taking into consideration our priority
  double set_trail_angle = 0.0;
  double set_trail_range = 0.0;

  // convert from uint to int
  int own_target_priority_int = static_cast<int>(m_own_target_priority);

  if (own_target_priority_int == 1){
    // we are the closest
    set_trail_angle = best_trail_angle;
    set_trail_range = m_primary_trail_range; 

  } else {
    // adjust
    double alternating_factor = 0.2;
    
    set_trail_angle = best_trail_angle;
    
    int offset = own_target_priority_int / 2;
    double angle_diff = static_cast<double>( offset ) * m_group_trail_angle_offset;
    
    if (own_target_priority_int % 2){
      set_trail_angle = best_trail_angle + angle_diff;
      set_trail_range = m_primary_trail_range + (1.0 + alternating_factor) * m_group_trail_range_offset;
    } else {
      set_trail_angle = best_trail_angle - angle_diff;
      set_trail_range = m_primary_trail_range + (1.0 - alternating_factor) * m_group_trail_range_offset;
    }

    
  }

  set_trail_angle = angle360(set_trail_angle);
  
  // update
  Notify("TRAIL_INFO", "contact=" + m_own_target);
  Notify("TRAIL_INFO", "trail_range=" + doubleToString(set_trail_range));
  Notify("TRAIL_INFO", "trail_angle=" + doubleToString(set_trail_angle));
  Notify("TRAIL_INFO", "trail_angle_type=absolute"); 
 
  return;
}


double GroupComboAlloc::getCostToIntercept(std::string teammate, double intrd_x, double intrd_y)
{
       
  // sanity check if contacts are in map
  if (m_node_rec_map.count(teammate) == 0)
    return 10000.0;

  // get info
  double friendly_x, friendly_y, friendly_hdg;
  friendly_x = m_node_rec_map[teammate].getX();
  friendly_y = m_node_rec_map[teammate].getY();
  friendly_hdg = m_node_rec_map[teammate].getHeading();

  // Compute dist
  double dist = distPointToPoint(friendly_x, friendly_y, intrd_x, intrd_y);
  
  return(dist); 
}


//-------------------------------------------
// Procedure: buildCostMap()
//            builds out m_cost_map for all intercepting
//            agents to tackle all targets. 

void GroupComboAlloc::buildCostMap(bool include_own, bool rebuild, std::set<std::string> allocated_samples){


  /// If just removing the entries related to the already
  // alocated samples, then don't rebuild
  if (!rebuild){

    std::map<std::string, std::map<std::string, double> >::iterator itm;
    for (itm = m_cost_map.begin(); itm != m_cost_map.end(); itm++){
      // remove all the allocated samples
      std::map<std::string, double>::iterator it;
      for (it = itm->second.begin(); it != itm->second.end();){
	if (allocated_samples.count(it->first) > 0){
	  itm->second.erase(it++);
	} else {
	  ++it;
	}
      }
    }
    return;
  }

  // otherwise, we need to rebuild the map
  m_cost_map.clear();

  std::map<std::string, double> agent_costs;

  if (include_own){
    agent_costs = getAgentCosts(m_vname, allocated_samples);

    // add to the larger map
    m_cost_map[m_vname] = agent_costs; 
  }
  
  // now add the others
  std::set<std::string>::iterator it2;
  for (it2 = m_pop_state_map[m_active_option_name].begin(); it2 != m_pop_state_map[m_active_option_name].end(); it2++){
    agent_costs.clear();
    agent_costs = getAgentCosts(*it2, allocated_samples);
  
    // add to the larer map
    m_cost_map[*it2] = agent_costs; 
  }

  /*
  std::cout << "Made cost map, it is:" << std::endl;
  std::map<std::string, std::map<std::string, double>>::iterator it3;
  for (it3 = m_cost_map.begin(); it3 != m_cost_map.end(); it3++){
    std::cout << " @" << it3->first << std::endl;
    std::map<std::string, double>::iterator it4;
    for (it4 = it3->second.begin(); it4 != it3->second.end(); it4++){
      std::cout << "   ->" << it4->first << ", " << doubleToString(it4->second) << std::endl;
    }
  }
  */
  return;
}




//-------------------------------------------
// Procedure: getAgentCosts()
//            calculates the individual cost for the agent
//            to intercept each intruder

std::map<std::string, double>  GroupComboAlloc::getAgentCosts(std::string vname, std::set<std::string> allocated_samples)
{

  double intrd_x, intrd_y, cost;
  std::string intrd_name = "to-be-set";

  // Add additional costs if specified:
  double extra_cost = 0.0;
    
  std::map<std::string, double>::iterator it_cost = m_agent_extra_cost.find(vname);
  if ( it_cost != m_agent_extra_cost.end())
    extra_cost = m_agent_extra_cost[vname];
  
  it_cost = m_agent_already_assigned_cost.find(vname);
  if (it_cost != m_agent_already_assigned_cost.end())
    extra_cost += m_agent_already_assigned_cost[vname];

 
  std::map<std::string, double> agent_costs;
  std::set<std::string>::iterator it;
    
  for (it = m_intruder_set.begin(); it !=m_intruder_set.end(); it++){
    // get cost to intercept this enemy

    // Check if already allocated
    if (allocated_samples.count(*it) > 0)
      continue;
    
    if(m_node_rec_map.count(*it) == 0){
      // get it from the intercept estimate
      
      // check the list of estimates
      bool found_estimate = false;
      std::list<std::string>::iterator lit;
      for (lit = m_intercept_estimates.begin(); lit != m_intercept_estimates.end(); lit++){
	parseInterceptEstimate(*lit, intrd_name, intrd_x, intrd_y);
	if (intrd_name == *it){
	  // we found a record
	  // mark it and stop
	  found_estimate = true;
	  break;
	}
      }
      if (!found_estimate){
	reportRunWarning("Cannot find location info to intercept this contact: " + *it);
      }
      
    } else {
      // we have it in the node record
      intrd_x = m_node_rec_map[*it].getX();
      intrd_y = m_node_rec_map[*it].getY();
      
    }
    cost = getCostToIntercept(vname, intrd_x, intrd_y);

    agent_costs[*it] = cost + extra_cost;
    
  }

  return(agent_costs);
  
}


//-----------------------------------------
// Procedure:  builds the dist matrix from the current
//             cost map.  Also updates two important vectors
//             m_intruder_vec, and m_unalloc_teammates_vec with the
//             if the teammate/intruder is still unallocated

std::vector<std::vector<double>> GroupComboAlloc::getDistMatrixFromCostMap()
{

  std::vector<std::vector<double>> new_matrix;
  std::vector<std::vector<double>> empty_matrix;  // returned on error
  
  // first update the ordered vector of the set of intruders
  // Just build from the cost map
  std::vector<std::string> new_intruder_vec;
  std::map<std::string, std::map<std::string, double>>::iterator cm_it;
  cm_it = m_cost_map.begin();
  if (cm_it != m_cost_map.end()){
    // get the names of all the intruders from this map entry
    std::map<std::string,double> first_agent_costs = cm_it->second;
    std::map<std::string, double>::iterator it2;
    for (it2 = first_agent_costs.begin(); it2 != first_agent_costs.end(); it2++){
      new_intruder_vec.push_back(it2->first); 
    }
  }
  
  m_intruder_vec = new_intruder_vec;
  Notify("INTRUDER_VEC", svectorToString(m_intruder_vec));

  // second update the ordered vector of the set of unallocated teammates.
  // this will change as the agents are assigned and there is no longer
  // vector of costs for them in the cost map
  std::vector<std::string> new_unalloc_teammates_vec;

  // Just build from the cost map, since we have already checked if
  // each agent (and ourselves) is participating
  for (cm_it = m_cost_map.begin(); cm_it != m_cost_map.end(); cm_it++){
    new_unalloc_teammates_vec.push_back(cm_it->first); 
  }
    
  m_unalloc_teammates_vec = new_unalloc_teammates_vec;
  Notify("UNALLOC_TEAMMATES_VEC", svectorToString(m_unalloc_teammates_vec));
  
  // Ok, now build the Dist Matrix
  // at each step we check that the map contains the keys we are looking
  // for.  This prevents unauthorized memory acces

  std::vector<double> cost_vec;
  
  // for each teammate get the vector of costs
  for (unsigned int i = 0; i < m_unalloc_teammates_vec.size(); i++){
    cost_vec.clear();
    
    //    std::cout << "    ------ i=" << std::to_string(i) << " ------  " << std::endl;
    // sanity check
    if (m_cost_map.count(m_unalloc_teammates_vec[i]) == 0){
      //std::cout << " Did not find teammate "<< m_unalloc_teammates_vec[i] << std::endl;
      return (empty_matrix);
    }
    
    for ( unsigned int j = 0; j < m_intruder_vec.size(); j++){
      //      std::cout << "      ------ j=" << std::to_string(j) << " ------  " << std::endl;
      
      // another sanity check
      if (m_cost_map[m_unalloc_teammates_vec[i]].count(m_intruder_vec[j]) == 0){
	//std::cout << " Did not find intruder "<< m_intruder_vec[j] << std::endl;
	return (empty_matrix);
      }
      
      //            get the cost map from the ith unalloc teammate
      //                          |                     and get the cost to
      //                          |                     intercept the jth intruder
      //                          |                             |
      //                          *                             *
      double cost = m_cost_map[m_unalloc_teammates_vec[i]][m_intruder_vec[j]];      
      cost_vec.push_back(cost); 
         
    }
    // now cost_vec is the cost for the ith teammate, so add it
    new_matrix.push_back(cost_vec); 
  }

  // print it for debugging
  /*
  std::cout << " $$$ New Cost matrix  $$$ " << std::endl;
  std::cout << "          Targets:    "; 
  for ( unsigned int j = 0; j < m_intruder_vec.size(); j++){
    std::cout <<  m_intruder_vec[j] << ",  "; 
  }
  std::cout << std::endl;
  
  for (unsigned int i = 0; i < new_matrix.size(); i++){
    std::cout << "agent = " << m_unalloc_teammates_vec[i] << " Cost Vec: "; 

    for (unsigned int j = 0; j < new_matrix[i].size(); j++){
      std::cout << doubleToStringX(new_matrix[i][j], 2) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  */
  return(new_matrix);  
}


//----------------------------------------
// Procedure:  postAssignmentsMap()
//
void GroupComboAlloc::postAssignmentsMap(bool include_own, double total_costs) {

  std::map<std::string, std::vector<std::string>>::iterator it1;
  std::vector<std::string>::iterator it2;

  std::string str_out = "";
  bool found_own_name = false;
  bool unassigned_target = false;
  
  for (it1 = m_assignments_map.begin(); it1 != m_assignments_map.end(); it1++){
    str_out += ";Target=" + it1->first;

    str_out += ":Assigned=";
    bool found_at_least_one = false;
    for (it2 = it1->second.begin(); it2 != it1->second.end(); it2++) {
      str_out += *it2;
      str_out += ",";
      found_at_least_one = true;
      if (*it2 == m_vname)
	found_own_name = true;
    }

    if(found_at_least_one){
      // remove the last comma
      str_out = str_out.substr(0, str_out.size()-1);
    } else {
      // We have an unassigned target/sample, 
      unassigned_target = true;
    }
  }

  // remove the leading ;
  if (str_out.size() > 0){
    str_out = str_out.substr(1, str_out.size());
  }

  //std::cout << str_out << std::endl;
  
  bool actually_participating = (m_curr_option == m_active_option_name);
  
  // Post to one of three options
  // ASSIGNMENTS  (we are in fact participating, and this assignment
  //               was calculated including our own participation.
  // ASSIGNMENTS_WITH_PARTICIPATION
  // ASSIGNMENTS_WITHOUT_PARTICIPATION

  if ((include_own) && (actually_participating)){
    Notify("ASSIGNMENTS", str_out);
  }

  if (include_own){
    Notify("ASSIGNMENTS_WITH_PARTICPATION", str_out);
    Notify("COST_WITH_PARTICIPATION", total_costs);
  } else {
    Notify("ASSIGNMENTS_WITHOUT_PARTICPATION", str_out);
    Notify("COST_WITHOUT_PARTICIPATION", total_costs);
  }


  // Clear out some postings if we are not in the list
  // and we are actually participating and the list
  // was calculated with our participate in mind

  if ((!found_own_name) && (actually_participating) && (include_own)){
    Notify("OWN_TARGET", "none");
    Notify("OWN_TARGET_PRIORITY", to_string(1001));
  }

  if (actually_participating){
    // Post if we found an unassigned target/sample
    Notify("UNASSIGNED_SAMPLE",boolToString(unassigned_target));
  }
    
 
  return;

}



int GroupComboAlloc::getTotalTeamSize(bool include_own)
{
  int team_size = 0;
  if (m_pop_state_map.count(m_active_option_name) > 0)
    team_size += m_pop_state_map[m_active_option_name].size();
  
  if (include_own)
    team_size += 1;
  
  return( team_size );
}
