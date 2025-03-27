/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: MIT Dept of Mechanical Eng                           */
/*    FILE: JoustGenerator.h                                     */
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

#ifndef JOUST_GENERATOR_HEADER
#define JOUST_GENERATOR_HEADER

#include <vector>
#include <string>
#include "XYPolygon.h"
#include "XYFormatUtilsPoly.h"

class JoustGenerator
{
 public:
  JoustGenerator();
  virtual ~JoustGenerator() {}

  // Setters
  bool   setCircle(std::string);
  bool   setPickAmt(std::string);
  bool   setAngMinDiff(std::string);
  bool   setAngMaxDiff(std::string);
  bool   setMaxTries(std::string);

  bool   setHdgSnap(std::string);
  bool   setSpdSnap(std::string);
  bool   setPtSnap(std::string);
  bool   setSpdConfig(std::string);
  void   setVerbose(bool v) {m_verbose=v;}
  void   seedRandom();

  // Getters
  unsigned int amt() const  {return(m_pick_amt);}
  unsigned int size() const {return(m_pick_pos_x.size());}

  double getPosX(unsigned int) const;
  double getPosY(unsigned int) const;
  double getPosH(unsigned int) const;
  double getDestX(unsigned int) const;
  double getDestY(unsigned int) const;
  double getSpeed(unsigned int) const;

  double getCenterX() const   {return(m_circ_x);}
  double getCenterY() const   {return(m_circ_y);}
  double getCenterRad() const {return(m_circ_rad);}

  // Actions
  bool   pick();

 protected:
  bool pickPositions();
  void pickHeadingVals();
  void pickSpeedVals();
  void pickCircle();

 protected: // Config variables
  unsigned int m_pick_amt;
  unsigned int m_max_tries;
  double       m_ang_min_diff;
  double       m_ang_max_diff;
  double       m_min_range;
  
  bool         m_verbose;

  double       m_spd_val1;
  double       m_spd_val2;

  double       m_circ_x;
  double       m_circ_y;
  double       m_circ_rad;
  bool         m_circ_set;
  
protected: // State variables
  double       m_pt_snap;
  double       m_hdg_snap;
  double       m_spd_snap;
  
  // The chosen positions regardless of how chosen
  std::vector<double>       m_pick_pos_x;
  std::vector<double>       m_pick_pos_y;
  std::vector<double>       m_pick_pos_h;

  std::vector<double>       m_pick_dest_x;
  std::vector<double>       m_pick_dest_y;
  std::vector<double>       m_pick_speeds;
};

#endif 

