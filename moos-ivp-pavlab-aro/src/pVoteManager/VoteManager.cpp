/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: VoteManager.cpp                                 */
/*    DATE: 22 Feb 2025                                     */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "VoteManager.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

VoteManager::VoteManager()
{
  m_pop_size = 1;
  m_min_vote_interval = 1;
}

//---------------------------------------------------------
// Destructor

VoteManager::~VoteManager()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool VoteManager::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if(key == "CONTACTS_LIST") {
      if ( !handleContactsList(msg.GetString()) )
	reportRunWarning("Unhandled Mail: " + key);

    } else if (key == "NEW_MISSION_VOTE") {
      if ( !handleNewMissionVote(msg.GetString(), msg.GetTime()) )
	reportRunWarning("Unhandled Mail: " + key);
    } else if (key == "MISSION_VOTE_REQ") {
      if ( !handleMissionVoteRequest(msg.GetString(), msg.GetTime()) )
	reportRunWarning("Unhandled Mail: " + key);
    } else if (m_input_vals.count(key) > 0){
      m_input_vals[toupper(key)] = msg.GetDouble();
      m_input_times[toupper(key)] = msg.GetDouble();
      
    } else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool VoteManager::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool VoteManager::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // set up new starts
  if (m_new_starts_pending.size() > 0 )
    processPendingStarts();

  // Do our own bookkeeping
  // Check all vote records for:
  //  1. Add own votes if needed
  addOwnData(); 
  //  2. Ship out if all own information is good.
  postAnyVotes(); 
  //  3. If the vote record is all complete, then
  //     post the results and clear the record.
  // finishAnyVotes();  




    
  // Do your thing here!
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool VoteManager::OnStartUp()
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
    if(param == "min_vote_interval") {
      handled = setPosDoubleOnString(m_min_vote_interval, value);
    }
    else if(param == "m_stale_input_thresh") {
      handled = setPosDoubleOnString(m_stale_input_thresh, value);
    }
    else if(param == "pop_size") {
      handled = setIntOnString(m_pop_size, value);
      handled = handled && (m_pop_size > 0); 
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void VoteManager::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("CONTACTS_LIST", 0);
  Register("NEW_MISSION_VOTE",0); 
}


//-----------------------------------------------------
// Handle Contact List Report
//
bool VoteManager::handleContactsList(std::string msg)
{
  // parse message
  std::set<std::string> new_contact_set;
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i=0; i<svector.size(); i++) {
    new_contact_set.insert(svector[i]);
  }
  m_contacts = new_contact_set;
  return(true);
 }

//-----------------------------------------------------
// Handle Mission Vote
//    Example: NEW_MISSION_VOTE=UTIL1,UTIL2,UTIL3. 
bool VoteManager::handleNewMissionVote(std::string msg, double time )
{
  // parse message
  std::set<std::string> new_vote_vars;
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i=0; i<svector.size(); i++) {
    if (svector[i] != "")
      new_vote_vars.insert(toupper(svector[i]));
  }

  if (new_vote_vars.size() ==0)
    return(false);

  // check if ok to post
  bool ok_to_post = checkOkToPost(new_vote_vars, time); 

  if (!ok_to_post)
    return(true);  // The request was correct, but too close to a
                   // a previous alert, or a pending alert
    
  // Ok this is a valid new request, so save it in the new pending
  // vote map.  First need to generate a hash
  std::string hash = tolower(hashAlphaNum(10));
  m_new_starts_pending[hash] = new_vote_vars;

  // and register for all the variables.
  std::set<std::string>::iterator its;
  for (its = new_vote_vars.begin(); its != new_vote_vars.end(); its++){
    Register(toupper(*its), 0);
    m_all_input_vars.insert(toupper(*its)); 
  }
  
  return(true);
}

//-----------------------------------------------------
// Handle Mission Vote Request
//    Example: MISSON_VOTE_REQ = vname=abe,hash=329foinf4,varval=UTIL1:10,varval=UTIL2:20
//            
bool VoteManager::handleMissionVoteRequest(std::string msg, double time)
{

  std::string this_msg_hash = ""; 
  std::vector<std::string> svector = parseString(msg, ',');
  std::vector<std::string> svector_orig = svector; 
  for (unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(biteStringX(svector[i], '='));
    string value = svector[i];
    if (param == "hash")
      this_msg_hash = value; 
  }

  bool ok = false;
  // Is there already a vote record with this hash?
  if (m_vote_map.count(this_msg_hash) > 0){
    ok = m_vote_map[this_msg_hash].handleIncomingMsg(svector_orig, time); 
  } else {
    // need to make a new vote record and add it to the map.
    VoteRecord new_vote_record;
    new_vote_record.setHash(this_msg_hash);
    ok = new_vote_record.handleIncomingMsg(svector_orig, time);
    if (ok)
      m_vote_map[this_msg_hash] = new_vote_record; 
  }
   
  
  return(ok); 
}




//------------------------------------------------------------
// Procedure: checkOkToPost(std::set<std::string> new_vote_vars, double time)
//            check is
bool VoteManager::checkOkToPost(std::set<std::string> new_vote_vars, double time)
{

  // First check that we have not recently sent a vote request out
  // for these exact vars, or recently recieved a vote request
  std::map<std::string, VoteRecord>::iterator it;
  bool ok_to_post = true; 
  for (it = m_vote_map.begin(); it != m_vote_map.end(); it++){
    if (it->second.isSameVars(new_vote_vars)){
      // check the time
      double dt = time - it->second.getVoteStartTime();
      if (dt< 0)
	dt *= -1.0;
      if (dt < m_min_vote_interval){
	ok_to_post = false;
	break; 
      }
    }
  }
  
  if (!ok_to_post)
    return(false);
  
  // Also check that we do not have a vote request pending for these
  // exact vars.
  std::map<std::string, std::set<std::string> >::iterator it2;
  for (it2 = m_new_starts_pending.begin(); it2 != m_new_starts_pending.end(); it2++){
    if (it2->second == new_vote_vars){
	ok_to_post = false;
	break; 
      } 
  }

  if (!ok_to_post)
    return(false);

  return(true); 
}


//------------------------------------------------------------
// Procedure: processPendingStarts()
//            Builds a new vote record and adds it to the
//            vote map.  The vote record is not sent at this
//            time because we likely do not have the information
//            because we have not registered for the messages. 

void VoteManager::processPendingStarts() 
{
  // Step 1.  Create a new Vote Record from the data
  //          in the new mission vote

  std::map<std::string, std::set<std::string> >::iterator it;
  for (it = m_new_starts_pending.begin(); it != m_new_starts_pending.end();){ 
    VoteRecord new_vote_record;
    new_vote_record.setHash(it->first);
    new_vote_record.setPopSize(m_pop_size);
    // don't set time until it ships out
    new_vote_record.setVoteVars(it->second); 

    m_vote_map[it->first] = new_vote_record;

    // erase 
    m_new_starts_pending.erase(it++); 
  }
  return; 
}


//------------------------------------------------------------
// Procedure: addOwnData()
//            Loop through the existing mission votes and
//            see if we need to add our own votes now that
//            we have registered for the variables, and the
//            data has maybe come in for them. 
void VoteManager::addOwnData() 
{
  
  std::map<std::string, VoteRecord>::iterator it;
  for (it = m_vote_map.begin(); it != m_vote_map.end(); it++){

    // do we need to add our votes?
    bool allOwnVotesSet = it->second.areAllVotesSet(m_host_community); 
    if (allOwnVotesSet)
      continue;
    

    // Ok, what are the votes we need to find values for?
    std::set<std::string> vote_vars = it->second.getVoteVars();
    std::set<std::string>::iterator sit;
    for (sit = vote_vars.begin(); sit != vote_vars.end(); sit++){
      std::string var_name = *sit; 
      // Is this var set?
      bool this_vote_set = it->second.isVoteSet(m_host_community, var_name); 
      if (this_vote_set)
	continue; 

      // Do we have fresh values for this var?
      if (m_input_vals.count(toupper(var_name)) >0){
	// is it fresh?
	double dt = MOOSTime() - m_input_times[toupper(var_name)];
	if (dt < m_stale_input_thresh){
	  // add it!
	  it->second.setVoteVal(m_host_community, var_name,
				m_input_vals[toupper(var_name)]); 
	}
      }
    } // end of for each var
  } // end of for each vote rec
  
  return; 
}



//------------------------------------------------------------
// Procedure: postAnyVotesFor(std::string vname)
//            checks if we need to post any votes from this voter
//            to our current contacts, posts if needed, and
//            then marks as posted
// 
void VoteManager::postAnyVotesFor(std::string voter)
{

  // For each vote record
  std::map<std::string, VoteRecord>::iterator it;
  for (it = m_vote_map.begin(); it != m_vote_map.end(); it++){

    // Step  Send vote if we have completed the record
    bool allOwnVotesSet = it->second.areAllVotesSet(voter);
    if (allOwnVotesSet){
      // who have I sent this data to?
      std::set<std::string> prev_sent_vehicles = it->second.getPrevVoteRecip(voter);
      
      // For each current contact 
      std::set<std::string>::iterator sit;
      for (sit = m_contacts.begin(); sit != m_contacts.end(); sit++){
	std::string cname = *sit; 
	
	// have I already sent this vote record for this vehicle to this contact?
	if (prev_sent_vehicles.count(cname) > 0)
	  continue; 

	NodeMessage node_message;
	node_message.setSourceNode(m_host_community);
	node_message.setDestNode(cname);
	node_message.setVarName("MISSION_VOTE_REQ");
	node_message.setStringVal(it->second.getSpec(voter));
	Notify("NODE_MESSAGE_LOCAL", node_message.getSpec());

	// Mark this vote was sent in the voting record
	it->second.addVoteSent(voter, cname); 
	
      }
    }

  }

  return; 
}


//------------------------------------------------------------
// Procedure: postAnyVotes()
//            Does two things:
//            1.  Sends our own votes to all neighbors in the con
//                contacts list.
//            2.  Relays any votes we have that our not ours to
//                others in the group.  The vote record keeps
//                track of who we sent what vote.
// 
void VoteManager::postAnyVotes()
{

  // Step 1.
  postAnyVotesFor(m_host_community);
  
  // Step 2.  For each current contact 
  std::set<std::string>::iterator sit;
  for (sit = m_contacts.begin(); sit != m_contacts.end(); sit++){
    std::string cname = *sit;
    postAnyVotesFor(cname); 
  }
  
  return; 
}





//------------------------------------------------------------
// Procedure: buildReport()

bool VoteManager::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




