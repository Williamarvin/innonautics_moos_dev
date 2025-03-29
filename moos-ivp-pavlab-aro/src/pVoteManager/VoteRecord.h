/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: VoteRecord.h                                    */
/*    DATE: 22 Feb 2025                                     */
/************************************************************/

/*  
 A new vote starts with simply a list of variables that is 
 sent to the vote manager via the variable NEW_MISSION_VOTE
 NEW_MISSON_VOTE = UTIL1, UTIL2, UTIL3
 
 A new vote record is not created until we have a complete set
 of our own votes, once this is done, we will send out a message
 to our neighbors with the following
 MISSON_VOTE_REQ = vname=abe,type=average,hash=329foinf4,varval=UTIL1:10,varval=UTIL2:20

 We also monitor MISSON_VOTE_REQ for new votes that are started
 others, and we will respond to them. 

*/


#ifndef VoteRecord_HEADER
#define VoteRecord_HEADER

#include <string>
#include <set>
#include <map>
#include <vector>
#include "MBUtils.h" // for biteStringX, toupper, tolower

class VoteRecord
{

 public:
  VoteRecord();
  ~VoteRecord();

  // setters
  void setPopSize(int val) {m_pop_size=val; return;}
  void setHash(std::string hash) {m_hash=hash; return;}
  void setVoteVars(std::set<std::string> vars) {m_vote_vars = vars; return;}
  void setVoteVal(std::string name, std::string var, double val);
  void setVoteStartTime(double val) {m_vote_start_time = val; return;}

  // getters
  double getVoteStartTime() const {return(m_vote_start_time);}
  std::set<std::string> getVoteVars() const {return(m_vote_vars);}

  bool isSameVars(std::set<std::string> &vars) const {return(vars == m_vote_vars);}
  std::set<std::string> getPrevVoteRecip(std::string voter_name); 

  // operational 
  bool handleIncomingMsg(std::vector<std::string> spec, double time);
  bool areAllVotesSet(std::string name);
  bool isVoteSet(std::string name, std::string var);

  std::string getSpec(std::string vname); 

  void addVoteSent(std::string voter_name, std::string contact_name); 

  bool allVotesIn(int pop_size);
  bool allVotesInByName(std::set<std::string> roster);
  
  // Function to overload
  virtual void onComplete(); 

 private:
  std::string m_hash;             // unique to each instance
  std::string m_type_name;        // type of vote i.e. average, min

  int m_pop_size;

  double m_vote_start_time; 
  std::set<std::string> m_vote_vars; // mostly for speed

  // First key on vehicle name, second key on var
  // so m_vote_val[name][var_name] = value; 
  std::map<std::string, std::map<std::string, double> > m_vote_val;

  // Record of who we have sent vote messages to.
  // keyed on vehicle name
  // so m_sent_record[name] = set of agents we have sent "name's"
  // record to 
  std::map<std::string, std::set<std::string> > m_sent_record; 

  
  

};
#endif 
