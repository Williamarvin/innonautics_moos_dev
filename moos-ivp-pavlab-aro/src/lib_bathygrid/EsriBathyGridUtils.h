/*****************************************************************/
/*    NAME: Nick Gershfeld                                       */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: EsriBathyGrid.h                                      */
/*    DATE: Nov 2nd 2021                                         */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef ESRI_BATHY_GRID_UTILS_HEADER
#define ESRI_BATHY_GRID_UTILS_HEADER

#include "EsriBathyGrid.h"
#include <string>
#include <math.h>  // atan2
#include "XYFormatUtilsSegl.h"

std::string getGridLabel(std::string grid_spec);
std::string getLawnmowerString(EsriBathyGrid grid, int vnum, int v_N, double degrees);
std::string getLawnmowerString2(EsriBathyGrid grid, int vnum, int v_N);


#endif
