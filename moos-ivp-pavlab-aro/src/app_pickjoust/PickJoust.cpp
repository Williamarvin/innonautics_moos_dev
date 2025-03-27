/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: PickJoust.cpp                                        */
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

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <sys/types.h>
#include <unistd.h>

#include "PickJoust.h"
#include "MBUtils.h"
#include "VQuals.h"
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "XYCircle.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

PickJoust::PickJoust()
{
  m_headers_enabled = false;
  m_file_overwrite  = false;
  m_reuse   = false;
  m_verbose = false;
}


//---------------------------------------------------------
// Procedure: setOutputFile()

bool PickJoust::setOutputFile(string file)
{
  if(!okFileToWrite(file)) {
    cout << "Not ok file to write:" << file << endl;
    return(false);
  }

  m_joust_file = file;
  return(true);
}


//---------------------------------------------------------
// Procedure: printChoices()

void PickJoust::printChoices()
{
  if(okFileToRead(m_joust_file) && m_reuse)
    return;

  FILE *fptr = 0;
  if(m_joust_file != "") {
    fptr = fopen(m_joust_file.c_str(), "w");
    if(!fptr) {
      cout << "Unable to open file: " << m_joust_file << endl;
      return;
    }
  }

  if(m_headers_enabled && fptr) {
    fprintf(fptr, "# Start Position seed file created by pickjoust\n");
    fprintf(fptr, "# %s\n", m_arg_summary.c_str());
  }

  string center_line;
  center_line += "cx=" + doubleToStringX(m_gen.getCenterX(),2);
  center_line += ",cy=" + doubleToStringX(m_gen.getCenterY(),2);
  center_line += ",rad=" + doubleToStringX(m_gen.getCenterRad(),2);
  if(fptr) 
    fprintf(fptr, "%s\n", center_line.c_str());
  else
    cout << center_line << endl;

  string line;
  for(unsigned int i=0; i<m_gen.size(); i++) {

    double px1 = m_gen.getPosX(i);
    double py1 = m_gen.getPosY(i);
    double px2 = m_gen.getDestX(i);
    double py2 = m_gen.getDestY(i);
    double hdg = m_gen.getPosH(i);
    double spd = m_gen.getSpeed(i);
    string vname  = getIndexVName1(i);
    string vcolor = getIndexVColor(i);

    line = "px1="  + doubleToStringX(px1,3);
    line += ",py1=" + doubleToStringX(py1);
    line += ",px2=" + doubleToStringX(px2);
    line += ",py2=" + doubleToStringX(py2);
    line += ",hdg=" + doubleToStringX(hdg);
    line += ",spd=" + doubleToStringX(spd);
    line += ",vname=" + vname;
    line += ",vcolor="+ vcolor;

    if(fptr) 
      fprintf(fptr, "%s\n",  line.c_str());
    else
      cout << line << endl;
  }
  
  if(fptr)
    fclose(fptr);
}

