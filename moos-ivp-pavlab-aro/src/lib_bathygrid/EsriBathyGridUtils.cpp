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

#include "EsriBathyGridUtils.h"

using namespace std;

//---------------------------------------------------------
// Procedure: getGridLabel(string grid_spec)
//            extract the label from a grid spec

string getGridLabel(string grid_spec)
{
  vector<string> svector = parseString(grid_spec, ',');
  unsigned int length = svector.size();
  if (length < 1){
    return("");
  }
  // Get the last entry which should be the label
  if ( biteStringX(svector[length-1], '=') == "label") {
    return(svector[length-1]);
  } else {
    return("");
  }
}


//---------------------------------------------------------
// Procedure getLawnmowerString(EsriBathyGrid grid, int vnum, int v_N)
//           return a lawnmower string for a vehicle

string getLawnmowerString(EsriBathyGrid grid, int vnum, int v_N, double degrees)
{
  // points = format=lawnmower, label=foxtrot, x=$(LAWNMIDDLEX), y=-135, height=170, width=$(LAWNWIDTH), lane_width=10, rows=north-south, startx=$(LAWNSTARTX), starty=$[Y], degs=0
  
  string lawn = "format=lawnmower, label=survey, x=";
  
  // get params
  double edge_min_x;
  double edge_min_y;
  double edge_max_x;
  double edge_max_y;

  grid.getBounds(edge_min_x, edge_min_y, edge_max_x, edge_max_y);

  double cell_size = grid.getCellSize();
  
  // adjust
  double min_x = edge_min_x;
  double min_y = edge_min_y;//cell_size/2;
  double max_x = edge_max_x;
  double max_y = edge_max_y;

  cout << "minx " << min_x << " maxx " << max_x << " miny " << min_y << " maxy " << max_y << endl;

  // if grid column number divides evenly into v_num
  int remainder = (int)((max_x-min_x)/cell_size) % vnum;

  cout << ((max_x-min_x)/cell_size) << endl;
  cout << remainder << endl;

  double x,y,height,width,start_x;
    
  if (remainder == 0) {

    // find x and width
    x = min_x+(max_x-min_x)*(1+2*v_N)/(2*vnum);
    width = (max_x-min_x)/vnum - cell_size;
    
  } else {

    cout << vnum/2 << " " << (remainder < vnum/2) << endl;
    
    if (v_N != (vnum-1)) { // not last vehicle, round columns to get 0 remainder

      // pretend that max is higher/lower so it is an even split
      if (remainder <= vnum/2)
	max_x = max_x + cell_size*(vnum-remainder); // round up
      else
	max_x = max_x - cell_size*(remainder); // round down

      // find x and width
      x = min_x+(max_x-min_x)*(1+2*v_N)/(2*vnum);
      width = (max_x-min_x)/vnum - cell_size;
    
    } else { // last vehicle, remaining columns

      // find left bound based on neighbor vehicle
      double adjusted_max;
      
      if (remainder <= vnum/2)
        adjusted_max = max_x + cell_size*(vnum-remainder); // round up
      else
	adjusted_max = max_x - cell_size*(remainder); // round down
      
      double adjusted_width = (adjusted_max-min_x)/vnum;
      double left_bound = min_x + adjusted_width*v_N;

      // find x and width
      x = left_bound + (max_x-left_bound)/2;//min_x+(max_x-min_x)*(1+2*v_N)/(2*vnum);      
      width = max_x-left_bound - cell_size;     

    }

  }

  // find y and height (same for all cases)
  y = (max_y+min_y)/2;
  height = (max_y-min_y)-cell_size;

  // find approximate start x
  start_x = min_x + width*v_N + cell_size*(1+v_N);

  // build string
  lawn += to_string(x) + ", y=" + to_string(y);
  lawn += ", height=" + to_string(height) + " , width=" + to_string(width);
  lawn += ", lane_width=" + to_string(cell_size);
  lawn += ", rows=north-south, startx=" + to_string(start_x) + ", starty=$[Y], degs=0";  

  return lawn;
}


//---------------------------------------------------------
// Procedure getLawnmowerString(EsriBathyGrid grid, int vnum, int v_N)
//           return a lawnmower string for a vehicle
//           grid is an EsriBathyGrid,
//           vnum is the total number of vehicles
//           v_N is the current vehicle (starting at 0)

string getLawnmowerString2(EsriBathyGrid grid, int vnum, int v_N)
{
  // Step 0.  Get the polygon border of the grid
  XYSegList seg_list = grid.getBorderSegList();
  // Check that we have a valid seglist
  if (seg_list.size() < 1)
    return("");  


  // Step 1.  Get the center x and y of this lawnmower
  //          Assume the order of the vertices in the
  //          seglist is:
  //          (0)----------------(3)
  //           |                  |
  //           |                  |
  //          (1)----------------(2)
  //
  double cell_size = grid.getCellSize();
  double x_mid_01 = ( seg_list.get_vx(0) + seg_list.get_vx(1) ) / 2;
  double y_mid_01 = ( seg_list.get_vy(0) + seg_list.get_vy(1) ) / 2;
  double x_mid_32 = ( seg_list.get_vx(3) + seg_list.get_vx(2) ) / 2;
  double y_mid_32 = ( seg_list.get_vy(3) + seg_list.get_vy(2) ) / 2;
  double mid_dist = sqrt( pow((x_mid_32 - x_mid_01),2) + pow((y_mid_32 - y_mid_01),2) );
  double lm_width = mid_dist / static_cast<double>(vnum);

  // Step 1.1  Round the width to the next grid width if needed
  //           This process ensures that the width of the lawnmower
  //           can be divided into even paths that have a lane width
  //           equal to the cell size. Use this in the calculation
  //           of the center point of the lawnmower pattern

  double dist_from_left_edge_to_center;
  
  //  determine if the number of cells is evenly divided by the
  //  number of vehicles
  int remainder = static_cast<int>(mid_dist/cell_size) % vnum;
  if (remainder == 0) {
    // no adjustment needed.
   dist_from_left_edge_to_center = lm_width/2.0 + lm_width * static_cast<double>(v_N);
  } else {
    // adjustment needed
    // pretend that max is higher/lower so it is an even split
    // Each vehicle will round the same

    // Compute a new width that is rounded up to the next
    // multiple of cell size.
    double new_lm_width;
    double lm_width_rnd_up = 0.0;
    while(lm_width_rnd_up < lm_width) {
      lm_width_rnd_up += cell_size;
    }
    
    if (remainder <= vnum/2) {
      // need to round up
      new_lm_width = lm_width_rnd_up;
     
    } else {
      // need to round down
      new_lm_width = lm_width_rnd_up - cell_size;
    }
    
    if (v_N != (vnum-1)) { // not last vehicle,
      // Use the rounded new lawnmower width in calcs.
      dist_from_left_edge_to_center = new_lm_width/2.0 + new_lm_width * static_cast<double>(v_N);
      lm_width = new_lm_width; // reset the lm width;
      
    } else { // last vehicle,
      // just cover the remaining space
      double dist_from_left_edge_to_right_edge_of_second_to_last_area = new_lm_width * (static_cast<double>(vnum) - 1.0);
      dist_from_left_edge_to_center = ( dist_from_left_edge_to_right_edge_of_second_to_last_area + mid_dist) / 2.0;
      // reset the lm width to be whatever is left over
      lm_width = mid_dist - dist_from_left_edge_to_right_edge_of_second_to_last_area;   
    }
    
  }
  
  double x_center = x_mid_01 + dist_from_left_edge_to_center / mid_dist * (x_mid_32 - x_mid_01);
  double y_center = y_mid_01 + dist_from_left_edge_to_center / mid_dist * (y_mid_32 - y_mid_01);

  // Step 2.  Get the width and height
  //          width is already calculated above
  //          this height value is not perfect, and assumes
  //          a relatively square grid that is rotated
  double x_at_dist_03 = seg_list.get_vx(0) + dist_from_left_edge_to_center / mid_dist * (seg_list.get_vx(3) - seg_list.get_vx(0));
  double y_at_dist_03 = seg_list.get_vy(0) + dist_from_left_edge_to_center / mid_dist * (seg_list.get_vy(3) - seg_list.get_vy(0));
  double x_at_dist_12 = seg_list.get_vx(1) + dist_from_left_edge_to_center / mid_dist * (seg_list.get_vx(2) - seg_list.get_vx(1));
  double y_at_dist_12 = seg_list.get_vy(1) + dist_from_left_edge_to_center / mid_dist * (seg_list.get_vy(2) - seg_list.get_vy(1));
  double lm_height = sqrt( pow( (x_at_dist_03 - x_at_dist_12), 2) + pow( (y_at_dist_03 - y_at_dist_12), 2) );
  
  
  // Step 3.  Get the start x, the start y 
  //          along the top edge from 0 to 3
  //          this is not rounded for now,  I don't think it will matter really. 
  //double dist_from_left_edge_to_right_lm_edge = lm_width * static_cast<double>(v_N);
  double dist_from_left_edge_to_right_lm_edge = dist_from_left_edge_to_center - lm_width / 2.0;
  double start_x = seg_list.get_vx(0) + dist_from_left_edge_to_right_lm_edge / mid_dist * (seg_list.get_vx(3) - seg_list.get_vx(0));
  double start_y = seg_list.get_vy(0) + dist_from_left_edge_to_right_lm_edge / mid_dist * (seg_list.get_vy(3) - seg_list.get_vy(0));


  // Step 4.  Get the rotation of the grid
  //          Average the slope of the top and bottom
  //          edge
  double angle_top = atan2( (seg_list.get_vy(3) - seg_list.get_vy(0) ) , ( seg_list.get_vx(3) - seg_list.get_vx(0) ) );
  double angle_bottom = atan2( (seg_list.get_vy(2) - seg_list.get_vy(1) ) , ( seg_list.get_vx(2) - seg_list.get_vx(1) ) );
  double angle = (angle_top + angle_bottom ) / 2.0;


  // Step 5.  Build the string
  // points = format=lawnmower, label=foxtrot, x=XXXXX, y=XXXXX, height=XXXXXX, width=XXXXXX, lane_width=10, rows=north-south, startx=XXXXX, starty=XXXX, degs=XXXX
  
  string lawn = "format=lawnmower, label=survey, x=";
  
  lawn += to_string(x_center) + ", y=" + to_string(y_center);
  lawn += ", height=" + to_string(lm_height) + ", width=" + to_string(lm_width - cell_size);
  lawn += ", lane_width=" + to_string(cell_size);
  lawn += ", rows=north-south, startx=" + to_string(start_x) + ", starty=" + to_string(start_y);
  lawn += ", degs=" + to_string(180 - angle*180.0/PI);  
  
  return(lawn); 
}
