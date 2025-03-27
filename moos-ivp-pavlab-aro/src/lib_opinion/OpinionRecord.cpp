/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OpinionRecord.cpp                               */
/*    DATE: April 5th, 2023                                 */
/************************************************************/


/* Class def for an opinion record for each neighbor.  The 
   assumed message format is 
   OPINION_MESSAGE=VNAME:GROUP:OPTION1_NAME,OPTION1_OPINION:OPTION2_NAME,OPTION2_OPINION
   
*/


#include "OpinionRecord.h"

//---------------------------------------------------------
// Constructor
OpinionRecord::OpinionRecord()
{
  m_vname = "";
  m_group = "";
  m_record_time = 0;

}


//---------------------------------------------------------
// Destructor
OpinionRecord::~OpinionRecord()
{
  
}


//---------------------------------------------------------
// Set Record from Msg
//   Example Msg -> VNAME:GROUP:OPTION1_NAME,OPTION1_OPINION:OPTION2_NAME,OPTION2_OPINION
bool OpinionRecord::setRecordFromMsg(std::string msg, double msg_time) {

  m_last_record_msg = msg; 
  m_record_time = msg_time; 
  // parse message
  std::vector<std::string> svector = parseString(msg, ':');
  unsigned int msg_size = svector.size();

  if (msg_size >= 1){
    if (svector[0] != "")
      m_vname = svector[0]; 
  } else { return(false);}

  if (msg_size >= 2) {
    if (svector[1] != "")
      m_group = svector[1]; 
  } else {return(false);}

  // Now get the opinions
  for (unsigned int i = 2; i < msg_size; i++) {
    std::string optn_opinion_pair = svector[i];
    std::string str_opinion;
    std::string optn_name;
    optn_name = tolower(biteStringX(optn_opinion_pair, '='));
    str_opinion = optn_opinion_pair;

    // Checking and converting
    double opinion;
    bool ok = setDoubleOnString(opinion, str_opinion);
    
    if ( (optn_name != "") && ok ) {
      // save this
      m_opinion_vals[optn_name] = opinion;  // overwrite if needed
      
    } else { return(false);}

  }

  return(true);
}


//--------------------------------------------------------
//   Check if an option exists
bool OpinionRecord::isOptionExist(std::string q_val)
{
  return( (m_opinion_vals.find(q_val) != m_opinion_vals.end()) );
}


//-------------------------------------------------------
//  Get the double value of the opinion for this option
bool OpinionRecord::getOpinionForOption(std::string q_val, double& val)
{
  // First check the option exists
  if ( !(this->isOptionExist(q_val)) )
    return(false);

  val = m_opinion_vals[q_val];
  return(true);
}


//--------------------------------------------------------
//  getOpinionsSquared
double OpinionRecord::getOpinionsSquared()
{
  double ret = 0.0;
  
  std::map<std::string, double>::iterator it;
  for (it = m_opinion_vals.begin(); it != m_opinion_vals.end(); it++) {
    ret += it->second * it->second;
  }

  // Normalize before returning
  // Assume only opinions about active options
  // are being sent. 
  unsigned int N_o_int = m_opinion_vals.size();
  if (N_o_int == 0)
    N_o_int = 1;  // don't divide by zero
  
  double N_o = static_cast<double>(N_o_int);

  return(ret/N_o);
}


// getStrongestOpinion
std::string OpinionRecord::getStrongestOpinion()
{
  std::string strongest_opinion_name = "";
  double highest_opinion = 0.0;  //Opinions are in the simplex

  std::map<std::string, double>::iterator it;
  for (it = m_opinion_vals.begin(); it != m_opinion_vals.end(); it++) {
    if (it->second >= highest_opinion) {
      strongest_opinion_name = it->first;
      highest_opinion = it->second; 
    }
  }
  return(strongest_opinion_name); 
}
