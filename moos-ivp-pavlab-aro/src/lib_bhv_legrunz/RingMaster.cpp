/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng MIT                           */
/*    FILE: RingMaster.cpp                                       */
/*    DATE: June 6th, 2023                                       */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/

#include <cmath>
#include <set>
#include <iostream>
#include "MBUtils.h"
#include "AngleUtils.h"
#include "RingMaster.h"

#ifndef M_PI
#define M_PI 3.1415926
#endif

using namespace std;

//-----------------------------------------------------------
// Constructor()

RingMaster::RingMaster()
{
  // Init config vars
  m_stale_thresh = 100; // seconds after which vname entry dropped
  m_coord_extrap = true; 

  // Init state vars
  m_curr_utc = 0;  

  m_center_dist  = -1;
  m_center_degs  = -1;
  m_guard_active = true;

  // generalized leg run extents (avg over self and others)
  m_glr_total_len = 0;
  m_glr_turn1_len = 0;
  m_glr_turn2_len = 0;
  m_glr_leg_len   = 0;

  // Init other state vars
  m_total_stale = 0;
}

//-----------------------------------------------------------
// Procedure: setCurrUTC()

void RingMaster::setCurrUTC(double dval)
{
  if(dval <= 0)
    return;
  m_curr_utc = dval;
}

//-----------------------------------------------------------
// Procedure: getOwnDegs()

double RingMaster::getOwnDegs()
{
  if(m_map_degs.count("ownship") != 0)
    return(m_map_degs["ownship"]);

  return(-1);
}

//-----------------------------------------------------------
// Procedure: getOwnDegsPerSec()

double RingMaster::getOwnDegsPerSec()
{
  if(m_map_spd.count("ownship") != 0)
    return(m_map_spd["ownship"]);

  return(-1);
}

//-----------------------------------------------------------
// Procedure: setDimensionsLR()

void RingMaster::setDimensionsLR(string vname, double leg,
				 double td1, double td2)
{
  // Sanity checks
  if(vname == "")
    return;
  if((leg <= 0) || (td1 <= 0) || (td2 <= 0))
    return;

  // Update dimensions for this vehicle
  m_map_td1[vname] = td1;
  m_map_td2[vname] = td2;
  m_map_leg[vname] = leg;

  updateGroupDims();
}

//-----------------------------------------------------------
// Procedure: updatePos()
//      Note: Incoming vehicle speed (vspd) is in degs/sec.
//            Speed is primarily used for extrapolating to
//            mitigage intermittent updates.
//            It may also be useful to know the "group" spd
//            at times.
//      Note: The vehicle distance is provided in "degrees".
//            This is a distance in meters that is normalized
//            to the range [0,360). We want to use a normalized
//            range anyway, but we also treat the distance as if
//            it were a point on a circle and use angle-wrap
//            logic to calculate an average across vehicles.
//      Note: UTC time is noted and stored so we can extrapolate
//            later, to mitigate intermittent udpates.

bool RingMaster::updatePosCN(string vname, double vspd,
			     double degs, double utc)
{
  if((degs < 0) || (utc <= 0))
    return(false);
  
  if(m_map_degs.count(vname) == 0)
    m_guard_active = false;
  
  m_map_degs[vname] = degs;
  m_map_utc[vname]  = utc;
  m_map_spd[vname]  = vspd;

  return(true);
}

//---------------------------------------------------------
// Procedure: updateCenter()

void RingMaster::updateCenter()
{
  checkForStale();

  cout << "RingMaster: updateCenter() (S)" << endl;
  
  // Sanity check
  if(m_map_degs.size() == 0)
    return;

  extrapolate();

  double s = 0;
  double c = 0;
  double ssum = 0;
  double csum = 0;
  double avg = 0;

  map<string,double>::iterator p;
  for(p = m_map_degs.begin(); p!=m_map_degs.end(); p++) {
    cout << p->first << " uc: degs:" << p->second << endl;
    double iheading = p->second;
    
    s = sin(iheading * M_PI / 180);
    c = cos(iheading * M_PI / 180);

    ssum += s;
    csum += c;    
  }

  avg = atan2(ssum, csum) * 180 / M_PI;

  avg = angle360(avg);

  m_center_degs = avg;
  m_center_dist = (avg / 360) * m_glr_total_len;
  cout << "RingMaster: updateCenter() (E)" << endl;
}


//---------------------------------------------------------
// Procedure: extrapolate()

void RingMaster::extrapolate()
{
  if(!m_coord_extrap)
    return;
  if(m_curr_utc <= 0)
    return;

  // Part 1: Extrapolate
  map<string, double>::iterator p;
  for(p=m_map_degs.begin(); p!=m_map_degs.end(); p++) {
    string vname = p->first;
    double xdegs = p->second; // last reported degs

    double spd = m_map_spd[vname];
    double utc = m_map_utc[vname];
    double elapsed = m_curr_utc - utc;

    if((spd > 0) && (utc > 0) && (elapsed > 0))
      xdegs += (spd * elapsed);
    m_map_degs[vname] = xdegs;
    m_map_utc[vname] = m_curr_utc; // mikerb
  }

  // Part 2: Ensure extrapolated distances are within the
  // range [0, 360). 
  for(p=m_map_degs.begin(); p!=m_map_degs.end(); p++) {
    string vname = p->first;
    double xdegs = p->second;

    xdegs = angle360(xdegs);
    m_map_degs[vname] = xdegs;
  }
}

//---------------------------------------------------------
// Procedure: checkForStale()

void RingMaster::checkForStale()
{
  if(m_curr_utc <= 0)
    return;

  // Part 1: Identify stale vehicles.
  set<string> stale_vnames;  
  map<string, double>::iterator p;
  for(p=m_map_utc.begin(); p!=m_map_utc.end(); p++) {
    string vname = p->first;
    double utc = p->second;

    if(vname != "ownship") {
      if((m_curr_utc - utc) > m_stale_thresh) {
	stale_vnames.insert(vname);
	m_total_stale++;
      }
    }
  }

  if(stale_vnames.size() == 0)
    return;
  
  // Part 2: remove stale vehicles
  set<string>::iterator q;
  for(q=stale_vnames.begin(); q!=stale_vnames.end(); q++) {
    string vname = *q;

    m_map_degs.erase(vname);
    m_map_spd.erase(vname);
    m_map_utc.erase(vname);
  
    m_map_td1.erase(vname);
    m_map_td2.erase(vname);
    m_map_leg.erase(vname);
  }

  updateGroupDims();
}

//-----------------------------------------------------------
// Procedure: updateGroupDims()

void RingMaster::updateGroupDims()
{
  // Sanity checks
  if((m_map_td1.size() == 0) || (m_map_td2.size() == 0))
    return;
  if(m_map_leg.size() == 0)
    return;
  
  // Update the average dimensions
  double glr_turn1_total = 0;
  double glr_turn2_total = 0;
  double glr_leg_total = 0;

  map<string,double>::iterator p;
  for(p=m_map_td1.begin(); p!=m_map_td1.end(); p++) {
    string vname = p->first;
    glr_turn1_total += m_map_td1[vname];
    glr_turn2_total += m_map_td2[vname];
    glr_leg_total += m_map_leg[vname];
  }
  
  m_glr_turn1_len = glr_turn1_total / (double)(m_map_td1.size());
  m_glr_turn2_len = glr_turn2_total / (double)(m_map_td2.size());
  m_glr_leg_len   = glr_leg_total / (double)(m_map_leg.size());
  m_glr_total_len = m_glr_turn1_len + m_glr_turn2_len;
  m_glr_total_len += (2 * m_glr_leg_len);
}

