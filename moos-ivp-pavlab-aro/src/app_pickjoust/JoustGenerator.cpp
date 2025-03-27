/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: MIT Dept of Mechanical Eng                           */
/*    FILE: JoustGenerator.cpp                                   */
/*    DATE: March 8th, 2024                                      */
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

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <sys/types.h>
#include <unistd.h>

#include "JoustGenerator.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "XYCircle.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

JoustGenerator::JoustGenerator()
{
  // By default, positions and headings are rounded to nearest integer
  m_pt_snap   = 0.1;
  m_hdg_snap  = 0.1;
  m_spd_snap  = 0.1;

  // When making random pts possibly enforce angular separation
  m_ang_min_diff = -1;
  m_ang_max_diff = -1;
  m_min_range    = -1;
  
  // When making random pts, num tries to find pt satisfying buffer
  m_max_tries = 10000;

  // Spd_val config params to support picking position speed vals
  m_spd_val1 = 0;
  m_spd_val2 = 0;

  // For Picking points on a circle
  m_circ_x = 0;
  m_circ_y = 0;
  m_circ_rad = 0;
  m_circ_set = false;

  // Total amount of points, destinations, speeds  to make
  m_pick_amt = 1;

  // Verbose output will show all created info on the command line
  m_verbose = false;
}


//---------------------------------------------------------
// Procedure: setPickAmt()

bool JoustGenerator::setPickAmt(string str)
{
  return(setPosUIntOnString(m_pick_amt, str));
}

//---------------------------------------------------------
// Procedure: setAngMinDiff()

bool JoustGenerator::setAngMinDiff(string str)
{
  bool ok = setDoubleRngOnString(m_ang_min_diff, str, 0, 179);
  if(!ok)
    return(false);

  // If ang_MAX_diff is in use, it should be at least 1 degree
  // greater than ang_min_diff.
  if((m_ang_max_diff > 0) && (m_ang_max_diff+1 <= m_ang_min_diff))
    m_ang_max_diff = m_ang_min_diff + 1;

  return(true);     
}

//---------------------------------------------------------
// Procedure: setAngMaxDiff()

bool JoustGenerator::setAngMaxDiff(string str)
{
  bool ok = setDoubleRngOnString(m_ang_max_diff, str, 1, 180);
  if(!ok)
    return(false);

  // If ang_MIN_diff is in use, it should be at least 1 degree
  // less than ang_max_diff.
  if((m_ang_min_diff > 0) && (m_ang_min_diff >= m_ang_max_diff-1))
    m_ang_min_diff = m_ang_max_diff - 1;

  return(true);     
}

//---------------------------------------------------------
// Procedure: setMaxTries()

bool JoustGenerator::setMaxTries(string str)
{
  return(setPosUIntOnString(m_max_tries, str));
}

//---------------------------------------------------------
// Procedure: setHdgSnap()

bool JoustGenerator::setHdgSnap(string str)
{
  return(setNonNegDoubleOnString(m_hdg_snap, str));
}

//---------------------------------------------------------
// Procedure: setSpdSnap()

bool JoustGenerator::setSpdSnap(string str)
{
  return(setNonNegDoubleOnString(m_spd_snap, str));
}

//---------------------------------------------------------
// Procedure: setPtSnap()

bool JoustGenerator::setPtSnap(string str)
{
  return(setNonNegDoubleOnString(m_pt_snap, str));
}

//---------------------------------------------------------
// Procedure: setSpdConfig
//     Notes: "1:5"  --> rand spd value in [1:5] chosen

bool JoustGenerator::setSpdConfig(string str)
{
  string low = biteStringX(str, ',');
  string hgh = str;
  if(!isNumber(low) || !isNumber(hgh))
    return(false);

  double dlow = atof(low.c_str());
  double dhgh = atof(hgh.c_str());
  if((dlow < 0) || (dhgh < 0))
    return(false);

  if(dlow > dhgh)
    return(false);

  m_spd_val1 = dlow;
  m_spd_val2 = dhgh;
  return(true);
}

//---------------------------------------------------------
// Procedure: setCircle()
//   Example: "x=23,y=43,rad=100

bool JoustGenerator::setCircle(string str)
{
  if(isQuoted(str))
    str = stripQuotes(str);

  string xstr = tokStringParse(str, "x");
  string ystr = tokStringParse(str, "y");
  string rstr = tokStringParse(str, "rad");

  bool ok = setDoubleOnString(m_circ_x, xstr);
  ok = ok && setDoubleOnString(m_circ_y, ystr);
  ok = ok && setPosDoubleOnString(m_circ_rad, rstr);

  m_circ_set = ok;
  return(ok);
}
  
//----------------------------------------------------------------
// Procedure: seedRandom 

void JoustGenerator::seedRandom()
{
  unsigned long tseed = time(NULL)+1;
  unsigned long pid = (long)getpid()+1;
  unsigned long seed = (tseed%999999);
  seed = ((rand())*seed)%999999;
  seed = (seed*pid)%999999;
  srand(seed);
}

//---------------------------------------------------------
// Procedure: getPosX()

double JoustGenerator::getPosX(unsigned int ix) const
{
  if(ix >= m_pick_pos_x.size())
    return(0);

  return(m_pick_pos_x[ix]);
}

//---------------------------------------------------------
// Procedure: getPosY()

double JoustGenerator::getPosY(unsigned int ix) const
{
  if(ix >= m_pick_pos_y.size())
    return(0);

  return(m_pick_pos_y[ix]);
}

//---------------------------------------------------------
// Procedure: getPosH()

double JoustGenerator::getPosH(unsigned int ix) const
{
  if(ix >= m_pick_pos_h.size())
    return(0);

  return(m_pick_pos_h[ix]);
}

//---------------------------------------------------------
// Procedure: getDestX()

double JoustGenerator::getDestX(unsigned int ix) const
{
  if(ix >= m_pick_dest_x.size())
    return(0);

  return(m_pick_dest_x[ix]);
}

//---------------------------------------------------------
// Procedure: getDestY()

double JoustGenerator::getDestY(unsigned int ix) const
{
  if(ix >= m_pick_dest_y.size())
    return(0);

  return(m_pick_dest_y[ix]);
}

//---------------------------------------------------------
// Procedure: getSpeed()

double JoustGenerator::getSpeed(unsigned int ix) const
{
  if(ix >= m_pick_speeds.size())
    return(0);

  return(m_pick_speeds[ix]);
}

//---------------------------------------------------------
// Procedure: pick()

bool JoustGenerator::pick()
{
  if(!m_circ_set) {
    cout << "Circle not set. Quitting." << endl;
    return(false);
  }

  if(m_verbose) {
    cout << "Generating " << m_pick_amt << " positions" << endl;
    cout << "speed val1:   " << m_spd_val1 << endl;
    cout << "speed val2:   " << m_spd_val2 << endl;
  }

  seedRandom();
  pickPositions();
  pickSpeedVals();

  if(m_pick_pos_x.size() != m_pick_amt)
    return(false);
  
  return(true);
}

//---------------------------------------------------------
// Procedure: pickPositions()

bool JoustGenerator::pickPositions()
{
  if(!m_circ_set)
    return(false);

  // ===========================================================
  // Part 1: Pick the initial positions
  // ===========================================================

  //cout << "ang_min_diff:" << m_ang_min_diff << endl;
  //cout << "ang_max_diff:" << m_ang_max_diff << endl;
  
  // Part 1A: handle case where all points are evenly separated
  if((m_ang_min_diff < 0) && (m_ang_max_diff < 0)) {
    double angle_delta = 360.0 / (double)(m_pick_amt);

    // First pick an arbitrary initial angle    
    double ang1 = ((double)(rand() % 3600)) / 10.0;

    //cout << "ang1: " << ang1 << endl;
    //cout << "angle_delta:" << angle_delta << endl;
    
    for(unsigned int i=0; i<m_pick_amt; i++) {
      double ang = angle360(ang1 + (i*angle_delta));
      double newx, newy;
      projectPoint(ang, m_circ_rad, m_circ_x, m_circ_y, newx, newy);
      m_pick_pos_x.push_back(newx);
      m_pick_pos_y.push_back(newy);
    }
  }
  // Part 1B: Randomly generated points
  else {
    vector<double> angles_so_far;
    for(unsigned int i=0; i<m_pick_amt; i++) {
      
      bool ok_point = false;
      for(unsigned int k=0; ((k<m_max_tries) && !ok_point); k++) {
	double ang = ((double)(rand() % 3600)) / 10.0;
	bool ok_min_angle = true;
	bool ok_max_angle = false;
	if(angles_so_far.size() == 0)
	  ok_max_angle = true;
	for(unsigned int j=0; j<angles_so_far.size(); j++) {
	  double diff1 = angleDiff(angles_so_far[j], ang);
	  double diff2 = angleDiff(angles_so_far[j], ang+180);
	  double diff = diff1;
	  if(diff2 < diff1)
	    diff = diff2;
	  
	  if((m_ang_min_diff >= 0) && (diff < m_ang_min_diff))
	    ok_min_angle = false;
	  if((m_ang_max_diff < 0) || (diff <= m_ang_max_diff))
	    ok_max_angle = true;
	}
	// Angle separation is ok, build the point and check min_sep
	if(ok_min_angle && ok_max_angle) {
	  double newx, newy;
	  projectPoint(ang, m_circ_rad, m_circ_x, m_circ_y, newx, newy);

	  if(m_min_range < 0)
	    ok_point = true;
	  else {
	    for(unsigned int j=0; j<m_pick_pos_x.size(); j++) {
	      double px = m_pick_pos_x[j];
	      double py = m_pick_pos_y[j];
	      double dist = hypot(newx - px, newy - py);
	      if(dist >= m_min_range)
		ok_point = true;
	    }
	  }
	  if(ok_point) {
	    m_pick_pos_x.push_back(newx);
	    m_pick_pos_y.push_back(newy);
	    angles_so_far.push_back(ang);
	  }
	}
      }
    }
  }

  // If we were unable to pick all pick_amt positions, then we clear
  // all of them. And creation of destinations is not done.
  if(m_pick_pos_x.size() != m_pick_amt) {
    m_pick_pos_x.clear();
    m_pick_pos_y.clear();
    m_pick_pos_h.clear();
    return(false);
  }


  // ===========================================================
  // Part 2: Pick the destination positions
  // ===========================================================
  for(unsigned int i=0; i<m_pick_amt; i++) {
    double x = m_pick_pos_x[i];
    double y = m_pick_pos_y[i];

    double hdg = relAng(x, y, m_circ_x, m_circ_y);
    double newx, newy;
    projectPoint(hdg, m_circ_rad, m_circ_x, m_circ_y, newx, newy);


    m_pick_pos_h.push_back(hdg);
    m_pick_dest_x.push_back(newx);
    m_pick_dest_y.push_back(newy);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: pickSpeedVals()

void JoustGenerator::pickSpeedVals()
{
  // Part 1: Sanity check
  if(m_pick_pos_x.size() != m_pick_amt) {
    if(m_verbose)
      cout << "No positions picked, so no speeds picked." << endl;
    return;
  }
  
  double range = m_spd_val2 - m_spd_val1;

  for(unsigned int i=0; i<m_pick_amt; i++) {
    if(range == 0)
      m_pick_speeds.push_back(m_spd_val1);
    else {
      int choices = (int)((100 * range));
      int rval = rand() % choices;
      double spd = m_spd_val1 + (double)(rval) / 100;
      m_pick_speeds.push_back(spd);
    }
  }
}

