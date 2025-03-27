/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: PickJoust.h                                          */
/*    DATE: Oct 28th, 2019                                       */
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

#ifndef PICKJOUST_HEADER
#define PICKJOUST_HEADER

#include <string>
#include "JoustGenerator.h"

class PickJoust
{
 public:
  PickJoust();
  virtual ~PickJoust() {}

  bool setOutputFile(std::string);
  void setFileOverWrite(bool v)     {m_file_overwrite=v;}
  void setArgSummary(std::string s) {m_arg_summary=s;}
  void setVerbose(bool v)           {m_verbose=v;}
  void enableHeaders()              {m_headers_enabled=true;}
  void setReuse()                   {m_reuse=true;}
  

  bool setCircle(std::string s)     {return(m_gen.setCircle(s));}  
  bool setPickAmt(std::string s)    {return(m_gen.setPickAmt(s));}
  bool setAngMinDiff(std::string s) {return(m_gen.setAngMinDiff(s));}
  bool setAngMaxDiff(std::string s) {return(m_gen.setAngMaxDiff(s));}
  bool setMaxTries(std::string s)   {return(m_gen.setMaxTries(s));}
  bool setHdgSnap(std::string s)    {return(m_gen.setHdgSnap(s));}
  bool setSpdSnap(std::string s)    {return(m_gen.setSpdSnap(s));}
  bool setPtSnap(std::string s)     {return(m_gen.setPtSnap(s));}
  bool setSpdConfig(std::string s)  {return(m_gen.setSpdConfig(s));}
  void seedRandom()                 {m_gen.seedRandom();}
  bool pick()                       {return(m_gen.pick());}
  void printChoices();
  
 protected: // Config variables
  JoustGenerator m_gen;
  
  bool         m_verbose;
  std::string  m_joust_file; 
  std::string  m_arg_summary;
  bool         m_headers_enabled;
  bool         m_file_overwrite;
  bool         m_reuse;
};

#endif 

