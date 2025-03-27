/*****************************************************************/
/*    NAME: Michael Benjamin, Tyler Paine                        */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: FldEvalConvoyEnginep                                 */
/*    DATE: July 25, 2021                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "FldEvalConvoyEngine.h"
#include "NodeRecordUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

FldEvalConvoyEngine::FldEvalConvoyEngine()
{
  m_recap_var      = "CONVOY_RECAP";
  m_stat_recap_var = "CONVOY_STAT_RECAP";
  m_spd_policy_var = "CONVOY_SPD_POLICY";
    
  // Internal State variables
  m_curr_time = 0;
  m_prev_time = 0;
  m_tstamp_first_recap = 0;

  m_recap_rcvd      = 0;
  m_stat_recap_rcvd = 0;
  m_spd_policy_rcvd = 0;

  m_convoy_length = -1;
}


//---------------------------------------------------------
// Procedure: setCurrTime()

void FldEvalConvoyEngine::setCurrTime(double curr_time)
{
  m_prev_time = m_curr_time;
  m_curr_time = curr_time;
}

//---------------------------------------------------------
// Procedure: updateMetrics()

void FldEvalConvoyEngine::updateMetrics()
{
  updateCoreMetrics();

}

//---------------------------------------------------------
// Procedure: setParam()

bool FldEvalConvoyEngine::setParam(string param, string value)
{
  bool handled = false;

  if(param == "recap_var") 
    handled = setNonWhiteVarOnString(m_recap_var, value);
  else if(param == "stat_recap_var") 
    handled = setNonWhiteVarOnString(m_stat_recap_var, value);
  else if(param == "spd_policy_var") 
    handled = setNonWhiteVarOnString(m_stat_recap_var, value);

  
  return(handled);
}

//---------------------------------------------------------
// Procedure: handleStatRecap()
//   Example: follower=henry,leader=abe,ideal_rng=40,compression=0.4

bool FldEvalConvoyEngine::handleStatRecap(string recap_str)
{
  m_stat_recap = string2ConvoyStatRecap(recap_str);
  m_stat_recap_rcvd++;

  handleStatRecapAlly(recap_str);
  return(true);
}

//---------------------------------------------------------
// Procedure: handleRecap()
//   Example: convoy_rng=17.4,vname=abe,rng_delta=-7.5,tail_rng=4.5,tail_ang=3.44,
//            mark_bng=46.41,almnt=49.84,set_spd=0.661,cnv_avg2=0.905,
//            cnv_avg5=0.867,cmode=close,mx=30.6,my=-11.8,mid=0,
//            tail_cnt=6,index=390

bool FldEvalConvoyEngine::handleRecap(string recap_str)
{
  if(m_tstamp_first_recap == 0)
    m_tstamp_first_recap = m_curr_time;
  
  m_recap = string2ConvoyRecap(recap_str);
  m_recap_rcvd++;

  // Update the range info with latest data
  updateRanges();

  
  return(true);
}

//---------------------------------------------------------
// Procedure: handleSpdPolicy()
//   Example: full_stop_rng=2,slower_rng=23,ideal_rng=25,faster_rng=27,
//            full_lag_rng=40,lag_spd_delta=2,max_compress=0.9

bool FldEvalConvoyEngine::handleSpdPolicy(string policy_str)
{
  m_spd_policy = string2ConvoySpdPolicy(policy_str);
  m_spd_policy_rcvd++;
  return(true);
}

//---------------------------------------------------------
// Procedure: handleStatRecapAlly()
//   Example: follower=henry,leader=abe,ideal_rng=40,compression=0.4

bool FldEvalConvoyEngine::handleStatRecapAlly(string stat_recap)
{
  if(stat_recap == "")
    return(false);
  
  string follower = tokStringParse(stat_recap, "follower", ',', '=');
  string leader   = tokStringParse(stat_recap, "leader", ',', '=');
  string idle     = tokStringParse(stat_recap, "idle", ',', '=');

  if((leader == "") || (follower == ""))
    return(false);

  if(idle != "true") { 
    string pairing = follower + "," + leader;    
    bool ok =  m_order_detector.addPairing(pairing);
  }
  else {
    m_order_detector.removeVehicle(follower);
    m_order_detector.removeVehicle(leader);
  }
    
  return(true);
}

//---------------------------------------------------------
// Procedure: handleNodeMessage()
//   Example: NAME=ida_GT,X=55.99,Y=-5.63,SPD=0,HDG=58,TYPE=kayak,
//            COLOR=green,GROUP=normal,MODE=PARK,ALLSTOP=ManualOverride,
//            INDEX=104,TIME=1630075311.08,LENGTH=4 


bool FldEvalConvoyEngine::handleNodeReport(string node_report_string)
{
  // Step 1.  Parse the message
  string vname;
  bool ground_truth = false;
  vector<string> node_report_vec = parseString(node_report_string, ',');

  for(unsigned int i=0; i<node_report_vec.size(); i++) {
    string param = tolower(biteStringX(node_report_vec[i], '='));
    string value = node_report_vec[i];

    if(param == "name") {
      vname = value;
      // Check if ground truth
      string simple_name = biteStringX( value, '_');
      if(value == "GT")
	ground_truth = true;
    }
    else if((param == "x") && (vname != "")) {      
      double x = strtod(value.c_str(), NULL);      
      if(ground_truth) 
	m_gt_x[vname] = x;
      else 
	m_nav_x_hydro[vname] = x;
    }
    else if((param == "y") && (vname != "")) {
      double y = strtod(value.c_str(), NULL);
      if(ground_truth)
	m_gt_y[vname] = y;
      else 
	m_nav_y_hydro[vname] = y;
    }
  }

  NodeRecord record = string2NodeRecord(node_report_string);
  if(record.valid()) {
    string vname = record.getName();
    m_map_records[vname] = record;
  }
  
  return(true);
}



//---------------------------------------------------------
// Procedure: getBool()

bool FldEvalConvoyEngine::getBool(string str) const
{
  //if(str == "on_tail")
  //  return(m_on_tail);
  //else if(str == "aligned")
  //  return(m_aligned);


  return(false);
}

//---------------------------------------------------------
// Procedure: getDouble()

double FldEvalConvoyEngine::getDouble(string str) const
{
  if(str == "convoy_length")
    return(m_convoy_length);
  //if(str == "time_on_tail")
  //  return(m_time_on_tail);
  //else if(str == "time_aligned")
  //  return(m_time_aligned);

  
  return(0);
}


//---------------------------------------------------------
// Procedure: getUInt()

unsigned int FldEvalConvoyEngine::getUInt(string str) const
{
  if(str == "recap_rcvd")
    return(m_recap_rcvd);
  else if(str == "stat_recap_rcvd")
    return(m_stat_recap_rcvd);
  else if(str == "spd_policy_rcvd")
    return(m_spd_policy_rcvd);

  return(0);
}

//---------------------------------------------------------
// Procedure: getStrBool()

string FldEvalConvoyEngine::getStrBool(string str) const
{
  bool val = getBool(str);
  return(boolToString(val));
}
    
//---------------------------------------------------------
// Procedure: getStrDouble()

string FldEvalConvoyEngine::getStrDouble(string str, int res) const
{
  if(res < 0)
    res = 0;

  double val = getDouble(str);
  return(doubleToString(val, res));
}

//---------------------------------------------------------
// Procedure: getStrDouble()

string FldEvalConvoyEngine::getStrUInt(string str) const
{
  unsigned int val = getUInt(str);
  return(uintToString(val));
}
    
//---------------------------------------------------------
// Procedure: getStrString()

string FldEvalConvoyEngine::getStrString(string str) const
{
  if(str == "recap_var")
    return(m_recap_var);
  if(str == "stat_recap_var")
    return(m_stat_recap_var);
  if(str == "spd_policy_var")
    return(m_spd_policy_var);
  
  return("");
}
    
//---------------------------------------------------------
// Procedure: updateCoreMetrics()

void FldEvalConvoyEngine::updateCoreMetrics()
{
  bool ok = calculateConvoyLength();
  // TODO
}



//---------------------------------------------------------
// Procedure: buildReport()

vector<string> FldEvalConvoyEngine::buildReport()
{
  vector<string> msgs;
  
  string s1 = " (" + getStrUInt("recap_rcvd") + ")";
  string s2 = " (" + getStrUInt("stat_recap_rcvd") + ")";
  string s3 = " (" + getStrUInt("spd_policy_rcvd") + ")";

  msgs.push_back("Configuration:");
  msgs.push_back("  recap_var:        " + m_recap_var +s1);
  msgs.push_back("  stat_recap_var:   " + m_stat_recap_var + s2);
  msgs.push_back("  spd_policy_var:   " + m_spd_policy_var + s3);

  msgs.push_back("");

  m_order_detector.findConvoy();
  if(m_order_detector.isValid() ) {
    msgs.push_back("Convoy Order:");
    string convoy_summary = m_order_detector.getConvoySummary();
    msgs.push_back(convoy_summary);
  }
  else 
    msgs.push_back("Convoy Order is NOT Valid");

  msgs.push_back("");
  msgs.push_back(m_order_detector.getPairsStr());
  msgs.push_back("Removals:" + uintToString(m_order_detector.getRemovalCnt()));
  msgs.push_back("");
  
  msgs.push_back("Convoy Ranges:");
  ACTable actab(3);
  actab << "Vehicle | Convoy Range | Delta";
  actab.addHeaderLines();

#if 0
  for (auto const &pair : m_latest_vehicle_range){
    string vn = pair.first;
    actab <<  pair.first << pair.second << m_latest_vehicle_range_delta.at(vn);
  }
  msgs.push_back(actab.getFormattedString());
  msgs.push_back("------------------------------");
  
  string s4 = getStrDouble("convoy_length", 4);
  if (m_convoy_length < 0)
    msgs.push_back("Convoy Length is Invalid");
  else
    msgs.push_back("Total Convoy Length = " + s4);
  msgs.push_back("");

  msgs.push_back("Localization Errors:");
  ACTable actab3(4);
  actab3 << "Vehicle | Error X | Error Y | Error Mag ";
  actab3.addHeaderLines();
  
  for (auto const &pair : m_nav_x_hydro){
    string vn = pair.first;
    string vn_gt = vn + "_GT";
    double error_x = 0.0;
    double error_y = 0.0;
    double error_mag = 0.0;
    
    if ( m_gt_x.find(vn_gt) != m_gt_x.end() ) {
      double error_x = abs( m_gt_x.at(vn_gt) - pair.second);
    }

    
    if ( ( m_gt_y.find(vn_gt) != m_gt_y.end() ) &&
	 ( m_nav_y_hydro.find(vn) != m_nav_y_hydro.end() ) ) {
      double error_y = abs( m_gt_y.at(vn_gt) - m_nav_y_hydro.at(vn) );
    }

    error_mag = sqrt( error_x * error_x + error_y * error_y);
    
    if ( (error_x != 0.0 ) && (error_y != 0.0) ) {
      actab3 << vn << error_x << error_y << error_mag;
    }
    
  }
  
  msgs.push_back(actab3.getFormattedString());
  msgs.push_back("---------------------------------------");

  
  msgs.push_back("Convoy Stability:");
  ACTable actab2(5);
  actab2 << "Vehicle | Max   | Time Since | Recent     | Osc ";
  actab2 << "        | Delta | Max Delta  | Delta Peak | Freq ";
  actab2.addHeaderLines();
  for (auto const &pair : m_max_vehicle_range_delta) {
    string vn = pair.first;
    double elapsed_time = m_curr_time - m_max_vehicle_range_delta_time.at(vn);
    if ( m_vehicle_range_delta_peak_freq.find(vn) != m_vehicle_range_delta_peak_freq.end() ) {
      actab2 <<  pair.first << pair.second <<  elapsed_time
	     << m_prev_vehicle_range_delta_peak_val.at(vn)
	     << m_vehicle_range_delta_peak_freq.at(vn);
    } else {
      actab2 <<  pair.first << pair.second << elapsed_time << "NA" << "NA";
    }
  }
  msgs.push_back(actab2.getFormattedString());
  msgs.push_back("---------------------------------------");
#endif
  
  return(msgs);
}



//------------------------------------------
// Procedure:  updateRanges()
void FldEvalConvoyEngine::updateRanges()
{
  // It is assumed that a convoy recap was just recieved
  // and processed as m_recap.

  string vname = m_recap.getStringValue("vname");
  double convoy_rng = m_recap.getConvoyRng();
  double convoy_rng_delta = m_recap.getConvoyRngDelta();
  double msg_time = m_recap.getTimeUTC();
  double convoy_rng_delta_running_avg = getRunningAvg(vname, convoy_rng_delta, msg_time);
  

  m_latest_vehicle_range[vname] = convoy_rng;
  m_latest_vehicle_range_delta[vname] = convoy_rng_delta;

  // Part 1.  Calculate the stability metric.
  //          Time between oscilations
 
  // Deterime how frequently this vehicle oscillates
  // by measuring the time between peaks in averaged
  // convoy range delta.  We need to average the date
  // to effectively low pass the signal, which will be
  // full of noisy peaks. 
  map<string, double>::iterator itr;
  itr = m_prev_vehicle_range_delta_val.find(vname);
  if (itr != m_prev_vehicle_range_delta_val.end() ) {

    
    // Check if we just passed a peak
    if ( convoy_rng_delta_running_avg < itr->second ) {
      
      // Just passed a peak.
      m_prev_vehicle_range_delta_peak_val[vname] = itr->second;

      // Now check that this is not the first peak detected
      if (m_prev_vehicle_range_delta_peak_time.find(vname)
	  != m_prev_vehicle_range_delta_peak_time.end() ) {
	// Calculate frequency
	double osc_period = msg_time - m_prev_vehicle_range_delta_peak_time.at(vname);

	if (osc_period > 0.0)
	  m_vehicle_range_delta_peak_freq[vname] = 1.0 / osc_period;
      }

      // Reset the peak time for next check
      m_prev_vehicle_range_delta_peak_time[vname] = msg_time;
      
    }
  }

  m_prev_vehicle_range_delta_val[vname] = convoy_rng_delta_running_avg;
  

  // Part 2.  Calculate time since last max range
  // Determine if this is a new maximum delta 
  map<string, double>::iterator itr2;
  itr2 = m_max_vehicle_range_delta.find(vname);
  if (itr2 == m_max_vehicle_range_delta.end() ) {
    // No entry yet
    m_max_vehicle_range_delta[vname] = convoy_rng_delta;
    m_max_vehicle_range_delta_time[vname] = msg_time;
  } else if ( itr2->second < convoy_rng_delta ) {
    m_max_vehicle_range_delta[vname] = convoy_rng_delta;
    m_max_vehicle_range_delta_time[vname] = msg_time;
  }
}

// ---------------------------------------
// Procedure: calculateConvoyLength()
//            Attempt to calculate convoy length.
//            Returns false if error, and also sets convoy
//            length to -1.

bool FldEvalConvoyEngine::calculateConvoyLength()
{
  vector<string> convoy_names = m_order_detector.getConvoyVector();
  std::reverse(convoy_names.begin(), convoy_names.end() );  // Returned in reverse order
  vector<string>::const_iterator p; 
  double convoy_length = 0.0;

  if (convoy_names.size() < 2){
    m_convoy_length = -1;
    return(false);
  }
  
  // Start from the second member in the convoy!
  for(p=next(convoy_names.begin()); p!=convoy_names.end(); p++)  {
    string convoy_member = *p;
    
    map<string, double>::iterator itr;
    itr = m_latest_vehicle_range.find(convoy_member);
    // First check we have range data from this member of the
    // convoy
    if(itr == m_latest_vehicle_range.end() ) {
      // data not found
      m_convoy_length = -1;
      return(false);
      
    } else {
      convoy_length += itr->second;
    }
  }

  m_convoy_length = convoy_length;

  return(true);
}


// ---------------------------------------
// Procedure: getConvoyReport()

string FldEvalConvoyEngine::getConvoyReport()
{
  m_order_detector.findConvoy();
  if(!m_order_detector.isValid())
    return("n/a");

  return(m_order_detector.getConvoySummary());
}

// ---------------------------------------
// Procedure: getMaxPairwiseRange()

double FldEvalConvoyEngine::getMaxPairwiseRange() const
{
  if(m_map_records.size() <= 1)
    return(0);

  double all_pairs_max_dist = 0;
  map<string, NodeRecord>::const_iterator p;
  for(p=m_map_records.begin(); p!=m_map_records.end(); p++) {
    double    max_dist = 0;
    string     vname1  = p->first;
    NodeRecord record1 = p->second;
    double x1 = record1.getX();
    double y1 = record1.getY();
    map<string, NodeRecord>::const_iterator q;
    for(q=m_map_records.begin(); q!=m_map_records.end(); q++) {
      string     vname2  = q->first;
      NodeRecord record2 = q->second;
      if(vname1 == vname2)
	continue;
      double x2 = record2.getX();
      double y2 = record2.getY();
      double dist = hypot(x1-x2, y1-y2);
      if(dist > max_dist)
	max_dist = dist;
    }
    if(max_dist > all_pairs_max_dist)
      all_pairs_max_dist = max_dist;
  }
  
  return(all_pairs_max_dist);    
}


//-----------------------------------------------------------
// Procedure: getTwoSecondRunningAvg(double val, double time)
//            // Taken from ConvoyBehavior

double FldEvalConvoyEngine::getRunningAvg(string vname, double val, double time)
{

  double max_age_of_queue = 5;  // seconds
  double averaging_window = 2;  // average over the last N seconds
  
  m_range_delta_hist[vname].push_front(val);
  m_range_delta_hist_tstamp[vname].push_front(time);

  // Part 1: Ensure queue is no older than 5 secs from when
  //         the message was published.
  double curr_time = time;
  bool done = false;
  while(!done && !m_range_delta_hist_tstamp[vname].empty()) {
    if((curr_time - m_range_delta_hist_tstamp[vname].back()) > max_age_of_queue) {
      m_range_delta_hist[vname].pop_back();
      m_range_delta_hist_tstamp[vname].pop_back();
    }
    else
      done = true;
  }


  // Part 2A: Build a copy of the spd queue only over N secs
  list<double> range_delta_hist  = m_range_delta_hist.at(vname);
  list<double> range_delta_hist_tstamp = m_range_delta_hist_tstamp.at(vname);
  done = false;
  while(!done && !range_delta_hist_tstamp.empty()) {
    if((curr_time - range_delta_hist_tstamp.back()) > averaging_window) {
      range_delta_hist.pop_back();
      range_delta_hist_tstamp.pop_back();
    }
    else
      done = true;
  }

  // Part 2B: Get the n second average
  double ave_val;
  double total_val = 0;
  unsigned int cnt = 0;
  list<double>::iterator p;
  for(p=range_delta_hist.begin(); p!=range_delta_hist.end(); p++) {
    double this_val = *p;
    total_val += this_val;
    cnt++;
  }
  if(cnt > 0)
    ave_val = total_val / (double)(cnt);
  else
    ave_val = 0;

  return(ave_val);
}
