/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: VoteRecord.cpp                                  */
/*    DATE: 22 Feb 2025                                     */
/************************************************************/

#include "VoteRecord.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

VoteRecord::VoteRecord()
{
  m_hash = "not_set";
  m_type_name = "";
  m_pop_size = 1;
  m_vote_start_time = 0.0; 
}

//---------------------------------------------------------
// Destructor

VoteRecord::~VoteRecord()
{
}


// -----------------------------------------------------------
// Handle Incoming Msg
//       This is when we already have a vote record started for this
//       hash and need to fill it in with responses from our neighbors
bool VoteRecord::handleIncomingMsg(std::vector<std::string> spec, double time)
{

  bool got_name = false;
  std::map<std::string, double> new_varval_map;
  std::string new_name = "";
  std::string new_type = ""; 
  
  for (unsigned int i=0; i<spec.size(); i++) {
    string param = tolower(biteStringX(spec[i], '='));
    string value = spec[i];

    // first check that the hash matches, just to be safe
    if (param == "hash"){
      if (m_hash != value)
	  return(false);
	  
    } else if (param == "vname"){
      new_name = value;
      got_name = true;
      
    } else if (param == "type"){
      if (value == "")
	return(false);
      new_type = value;
      
    } else if (param == "varval"){
      // varval=VAR_NAME:10.00
      std::string varname = toupper(biteStringX(value, ':'));
      std::string varval = value;
     
      if ((varname != "") && (varval != "")){
	double val_dbl = stod(varval);
	new_varval_map[varname] = val_dbl; 
      } else
	return(false); 
      
    }
  }
  
  if (got_name){
    m_type_name = new_type; 
    m_vote_val[new_name] = new_varval_map;
    return(true);
  } 
  return(false); 
}



// -----------------------------------------------------------
// areAllVotesSet
//             given a name, are all the votes present?

bool VoteRecord::areAllVotesSet(std::string name)
{

  // check if any are set ...
  if (m_vote_val.count(name) == 0)
    return(false);

  // now check for each one. 
  std::set<std::string>::iterator it;
  for (it = m_vote_vars.begin(); it != m_vote_vars.end(); it++){
    if (!isVoteSet(name, *it))
      return(false); 
  }
  return(true); 
}

// -----------------------------------------------------------
// areVotesSet
//             given a name, and a variable is the vote present?

bool VoteRecord::isVoteSet(std::string name, std::string var)
{

  // check if name exists for safety
  if (m_vote_val.count(name) == 0)
    return(false);

  // check if the var exists. 
  if (m_vote_val[name].count(var) == 0)
      return(false); 
 
  return(true); 
}



// -----------------------------------------------------------
// setVoteVal()
//           

void VoteRecord::setVoteVal(std::string name, std::string var, double val)
{

  // check if name exists
  if (m_vote_val.count(name) == 0){
    // make a new map
    std::map<std::string, double> new_map;
    new_map[var] = val;
    m_vote_val[name] = new_map;
  } else{
    // add a new entry in the existing map
    m_vote_val[name][var] = val; 
  }
  return; 
}


//-------------------------------------------------------------
// std::string getSpec(std::string vname)
//         Generates the spec with the vote record from the given
//         Agent. so for abe this is like
//          MISSON_VOTE_REQ = vname=abe,hash=329foinf4,varval=UTIL1:10,varval=UTIL2:20
//         If no record exists, then an empty string is returned
// 
std::string VoteRecord::getSpec(std::string vname)
{
  std::string spec = "";
  if(!areAllVotesSet(vname))
    return(spec); 

  spec += "vname=" + vname + ",";
  spec += "hash=" + m_hash + ",";
  spec += "type=" + m_type_name + ","; 

  // already checked it exists
  std::map<std::string, double>::iterator it;
  for (it = m_vote_val[vname].begin(); it != m_vote_val[vname].end(); it++){
    char buffer[10];
    sprintf(buffer, "%.*f", 6, it->second);
    spec += it->first + ":" + std::string(buffer) + ",";
  }

  // remove the last ",", will always be longer than 1
  spec = spec.substr(0, spec.size()-1);
 
  return(spec); 
}



//-----------------------------------------------------------
//  std::set<std::string> getSentDataTo(std::string vehicle)
//           Returns a set of names of neighbors that we have
//           already sent the vote data from this vehicle
//           

std::set<std::string> VoteRecord::getPrevVoteRecip(std::string voter_name)
{
  if (m_sent_record.count(voter_name) > 0)
    return(m_sent_record[voter_name]);

  // it is empty, so return an empty set
  std::set<std::string> recipients;
  return(recipients); 
}



//-----------------------------------------------------------
//  void addVoteSent(std::string voter_name, std::string contact_name)
//            Records that a record of the voter_name's vote was sent to
//            the contact_name
//
void VoteRecord::addVoteSent(std::string voter_name, std::string contact_name)
{
  
  if (m_sent_record.count(voter_name) > 0)
    m_sent_record[voter_name].insert(contact_name);
  else {
    std::set<std::string> new_set;
    new_set.insert(contact_name);
    m_sent_record[voter_name] = new_set; 
  }
  return; 
}




//-----------------------------------------------------------
//   bool allVotesIn(int population_size)
//          Returns true if all votes are in
bool VoteRecord::allVotesIn(int pop_size)
{
  return(pop_size == m_vote_val.size());
}

//-----------------------------------------------------------
//   bool allVotesInByName(std::set<std::string> roster)
//          Returns true if all votes are in
bool VoteRecord::allVotesInByName(std::set<std::string> roster)
{
  std::set<std::string>::iterator sit;
  for (sit = roster.begin(); sit != roster.end(); sit++){
    if (m_vote_val.count(*sit) == 0)
      return(false); 
  }
  return(true);
}


//-----------------------------------------------------------
//   bool onComplete()
//          Is called when all the votes are in and the
//          function needs to complete
void VoteRecord::onComplete()
{
  // do your thing here!
  
  return; 
}
