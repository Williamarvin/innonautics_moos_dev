/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: SpdModulator.cpp                                     */
/*    DATE: June 29th, 2021, broken out to separate class        */
/*    DATE: June 8th, 2023, Became generic speed modulator       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iostream>
#include "SpdModulator.h"
#include "MBUtils.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor()

SpdModulator::SpdModulator()
{
  m_full_stop_rng = 0;
  m_slower_rng    = 0;
  m_faster_rng    = 0;
  m_full_lag_rng  = 0;

  m_max_compression = 0.9;
  m_lag_speed_delta = -1;

  m_correction_mode = "unset";
}

//-----------------------------------------------------------
// Procedure: setFullStopRng()
//      Note: The full_stop_range is always negative. Negative
//            value indicates we have overshot the target position
//            and need to back off (slow down).

bool SpdModulator::setFullStopRng(double dval)
{
  if(dval >= 0)
    return(false);
  
  m_full_stop_rng = dval;
  return(true);
}

//-----------------------------------------------------------
// Procedure: setSlowerRng()
//      Note: The slower_range is always negative. Negative
//            value indicates we have overshot the target position
//            and need to back off (slow down).

bool SpdModulator::setSlowerRng(double dval)
{
  if(dval >= 0)
    return(false);
  
  m_slower_rng = dval;
  return(true);
}

//-----------------------------------------------------------
// Procedure: setFasterRng()
//      Note: The faster_range is always positive. Positive
//            value indicates we have undershot the target position
//            and need to get closer (speed up).

bool SpdModulator::setFasterRng(double dval)
{
  if(dval <= 0)
    return(false);
  
  m_faster_rng = dval;
  return(true);
}

//-----------------------------------------------------------
// Procedure: setFullLagRng()
//      Note: The full_lag_range is always positive. Positive
//            value indicates we have undershot the target position
//            and need to get closer (speed up).

bool SpdModulator::setFullLagRng(double dval)
{
  if(dval <= 0)
    return(false);
  
  m_full_lag_rng = dval;
  return(true);
}

//-----------------------------------------------------------
// Procedure: setMaxCompression()
//      Note: The max_compression is the greatest allowable
//            compression. By default it is 0.9. It can be
//            made smaller, all the way down to zero which
//            would make this policy uncompressible. It can
//            not be made more compressible than 0.9.
//      Note: If the given value is outside the allowable
//            range, it is mererly clipped.

void SpdModulator::setMaxCompression(double dval)
{
  m_max_compression = dval;
  
  if(m_max_compression < 0)
    m_max_compression = 0;
  if(m_max_compression > 0.9)
    m_max_compression = 0.9;
}

//-----------------------------------------------------------
// Procedure: setLagSpeedDelta()

void SpdModulator::setLagSpeedDelta(double dval)
{
  m_lag_speed_delta = dval;
}

//-----------------------------------------------------------
// Procedure: setParam()

bool SpdModulator::setParam(string param, string value) 
{
  bool handled = false;
  if(param == "full_stop_range")
    handled = setPosDoubleOnString(m_full_stop_rng, value);
  else if(param == "slower_range")
    handled = setPosDoubleOnString(m_slower_rng, value);
  else if(param == "faster_range")
    handled = setPosDoubleOnString(m_faster_rng, value);
  else if(param == "full_lag_range")
    handled = setPosDoubleOnString(m_full_lag_rng, value);

  else if(param == "lag_speed_delta")
    handled = setPosDoubleOnString(m_lag_speed_delta, value);

  return(handled);
}

//-----------------------------------------------------------
// Procedure: status()

string SpdModulator::status() 
{
  if(m_full_stop_rng <= 0)
    return("full_stop_rng not set or invalid");
  if(m_slower_rng < m_full_stop_rng) 
    return("slower_rng not set or invalid");
  if(m_full_lag_rng <=0)
    return("full_lag_rng not set or invalid");
  if(m_faster_rng < m_full_stop_rng) 
    return("faster_rng not set or invalid");
  
  return("ok");
}

//-----------------------------------------------------------
// Procedure: updateSetSpeed()
//     Notes: contact_rng: Straight-line (as the crow flies)
//                         distance to the contact
//            rng:  Length of tail + distance of ownship
//                         to the aft marker.
//            lead_speed:  Speed of the moving contact.
//
//   Purpose: Decide speed based on the policy and input params
// 
//   cn    fstop     slower       ideal      faster     full_lag
//   o       o          o           o           o          o
//      #1       #2           #3          #4         #5      #6


double SpdModulator::getSpdFromPolicy(double range,
				      double mark_speed)
{
  cout << "ggg: full_stop_rng: " << m_full_stop_rng << endl;
  cout << "ggg: slower_rng:   " << m_slower_rng << endl;
  cout << "ggg: faster_rng:   " << m_faster_rng << endl;
  cout << "ggg: full_lag_rng: " << m_full_lag_rng << endl;
  cout << "ggg: range: " << range << endl;
  cout << "ggg: mark spd: " << mark_speed << endl;
  // Case 1 Full stop now
  // e.g., full_stop_range=-40, range=-50
  if(range <= m_full_stop_rng) {
    m_correction_mode = "full_stop";
    cout << "ggg correction mode: " << m_correction_mode << endl;
    return(0);
  }

  // Case 2 Slower proportionally
  // e.g., full_stop_range=-40, slower_range=-25, range=-30
  if(range <= m_slower_rng) {
    m_correction_mode = "close";
    cout << "ggg correction mode: " << m_correction_mode<< endl;
    double span = m_slower_rng - m_full_stop_rng;
    if(span <= 0)
      return(0);

    double pct = (range - m_full_stop_rng) / span;
    return(pct * mark_speed);
  }

  // Case 3 and 4 Match contact speed
  if(range <= m_faster_rng) {
    m_correction_mode = "ideal";
    cout << "ggg correction mode: " << m_correction_mode << endl;
    return(mark_speed);
  }

  // Case 5 Speed up proportionally
  if(range <= m_full_lag_rng) {
    m_correction_mode = "far";
    cout << "ggg correction mode: " << m_correction_mode << endl;
    double span = m_full_lag_rng - m_faster_rng;
    if(span <= 0)
      return(mark_speed);
    else {
      double pct = (range - m_faster_rng) / span;
      return(mark_speed + (pct * m_lag_speed_delta));
    }
  }

  // Case 6 Speed up as much as allowable
  m_correction_mode = "full_lag";
  double full_lag_speed = mark_speed + m_lag_speed_delta;
  cout << "ggg correction mode: " << m_correction_mode << endl;
  return(full_lag_speed);      
}

//-----------------------------------------------------------
// Procedure: compress()

bool SpdModulator::compress(double pct)
{
#if 0
  if((pct < 0) || (pct > m_max_compression))
    return(false);
  if(status() != "ok")
    return(false);

  double slower_span = m_slower_rng - m_full_stop_rng;
  double idealA_span = m_ideal_rng - m_slower_rng;
  double idealB_span = m_faster_rng - m_ideal_rng;
  double faster_span = m_full_lag_rng - m_faster_rng;
  
  double new_slower_span = (1-pct) * slower_span;  
  double new_idealA_span = (1-pct) * idealA_span;  
  double new_idealB_span = (1-pct) * idealB_span;  
  double new_faster_span = (1-pct) * faster_span;  

  m_slower_rng = m_full_stop_rng + new_slower_span;
  m_ideal_rng  = m_slower_rng + new_idealA_span;
  m_faster_rng = m_ideal_rng + new_idealB_span;
  m_full_lag_rng = m_faster_rng + new_faster_span;
#endif
  return(true);
}


//-----------------------------------------------------------
// Procedure: getTerse()
//   Example: [5]    10--------20--------30      [50]   {2.0}

string SpdModulator::getTerse() const
{
  string str = "[" + doubleToStringX(m_full_stop_rng,2) + "]     ";;
  str += doubleToStringX(m_slower_rng,2);
  str += "----------";
  str += "(" + doubleToStringX(0,2) + ")";
  str += "----------";
  str += doubleToStringX(m_faster_rng,2) + "      ";
  str += "[" + doubleToStringX(m_full_lag_rng,2) + "]    ";
  str += "{" + doubleToStringX(m_lag_speed_delta,2) + "}";

  return(str);
}


//-----------------------------------------------------------
// Procedure: atIdealRng()

bool SpdModulator::atIdealRng(double rng)
{
  if(rng > m_faster_rng)
    return(false);
  if(rng < m_slower_rng)
    return(false);

  return(true);
}


//-----------------------------------------------------------
// Procedure: getSpec()

string SpdModulator::getSpec() const
{
  string spec = "full_stop_rng=";
  spec += doubleToStringX(m_full_stop_rng,2);

  spec += ",slower_rng=" + doubleToStringX(m_slower_rng,2);
  spec += ",faster_rng=" + doubleToStringX(m_faster_rng,2);
  spec += ",full_lag_rng=" + doubleToStringX(m_full_lag_rng,2);
  spec += ",lag_spd_delta=" + doubleToStringX(m_lag_speed_delta,2);
  spec += ",max_compress=" + doubleToStringX(m_max_compression,2);

  return(spec);
}

//-----------------------------------------------------------
// Procedure: string2SpdModulator()

SpdModulator string2SpdModulator(string msg)
{
  SpdModulator new_policy;

  vector<string> svector = parseString(msg, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    double dval = atof(value.c_str());
    
    if(param == "full_stop_rng")
      new_policy.setFullStopRng(dval);
    else if(param == "slower_rng")
      new_policy.setSlowerRng(dval);
    else if(param == "faster_rng")
      new_policy.setFasterRng(dval);
    else if(param == "full_lag_rng")
      new_policy.setFullLagRng(dval);
    else if(param == "lag_spd_delta")
      new_policy.setLagSpeedDelta(dval);
    else if(param == "max_compress")
      new_policy.setMaxCompression(dval);
  }

  return(new_policy);
}


