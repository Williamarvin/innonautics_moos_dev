/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: VoteManager.h                                   */
/*    DATE: 22 Feb 2025                                     */
/************************************************************/

#ifndef VoteManager_HEADER
#define VoteManager_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "VoteRecord.h"
#include "HashUtils.h"   // for hashAlphaNum()
#include "NodeMessage.h" // for NodeMessage

#include <string>
#include <set>
#include <vector>

class VoteManager : public AppCastingMOOSApp
{
 public:
   VoteManager();
   ~VoteManager();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

   // handling functions
   bool handleContactsList(std::string msg);
   bool handleNewMissionVote(std::string msg, double time);
   bool handleMissionVoteRequest(std::string msg, double time);  

 protected:
   void registerVariables();

   // main vote manager functions
   void processPendingStarts();
   void addOwnData();
   void postAnyVotes();
   void postAnyVotesFor(std::string voter);
   void finishAnyVotes(); 

   bool checkOkToPost(std::set<std::string> new_vote_vars, double time);

   // helper functions
   std::string getHash(std::string msg); 

 private: // Configuration variables
   int m_pop_size;
   double m_min_vote_interval;
   double m_stale_input_thresh; 
   
   

 private: // State variables
   std::set<std::string> m_contacts;

   // Keyed on hash
   std::map<std::string, std::set<std::string> > m_new_starts_pending;

   // We are worried about stale inputs for the votes.
   // Record of the time we recieved an input
   // Keyed on variable name initialliy set to 0.0
   std::map<std::string, double> m_input_times;
   std::map<std::string, double> m_input_vals;

   // set of the names of all input vars. 
   std::set<std::string> m_all_input_vars;

   // keyed on vote hash
   std::map<std::string, VoteRecord> m_vote_map;


   
};

#endif 
