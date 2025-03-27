/*****************************************************************/
/*    NAME: Tyler Paine,                                         */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ProxGrid.h                                           */
/*    DATE: Nov 12th 2024                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include "ProxGrid.h"


using namespace std;

//---------------------------------------------------------
// Constructor
ProxGrid::ProxGrid() {

}

//---------------------------------------------------------
// Constructor
ProxGrid::ProxGrid(XYConvexGrid new_grid, double no_data_value)
{
  m_grid = new_grid;
  m_no_data_value = no_data_value;
}


//----------------------------------------------------------
// getCellID
bool ProxGrid::getCellID(double x, double y, int &cell_id) {

  
  // find the appropriate cell.
  unsigned int cell_count = m_grid.size();
  for (int idx = 0; idx < cell_count; idx++) {

    bool contains_point = m_grid.ptIntersect(idx, x, y);
    if (contains_point) {
      cell_id = idx;
      return (true);
    }
  }
  // Was NOT able to find a suitable cell for this location
  return (false);
}


//-----------------------------------------------------------
// getCellXY
bool ProxGrid::getCellXY(int idx, double &x, double &y) {
  if (idx > m_grid.size())
    return (false);

  // get x and y data
  XYSquare cell_square = m_grid.getElement(idx);
  x = cell_square.getCenterX();
  y = cell_square.getCenterY();

  return (true);
}



//---------------------------------------------------------
// Procedure: updateCellValueIDX
//            Updates the cell based on index
//            with the new value 

bool ProxGrid::updateCellValueIDX(int idx,
				  double value, double variance) {

  // Skip if the cell is frozen
  if (m_frozen_cells.count(idx) > 0)
    return(true);
  
  if( m_grid.hasCellVar("val")){
    unsigned int val_idx = m_grid.getCellVarIX("val");
    m_grid.setVal( idx, value, val_idx);
  } else
    return (false);

  if( m_grid.hasCellVar("var")){
    unsigned int var_idx = m_grid.getCellVarIX("var");
    m_grid.setVal(idx, variance, var_idx);
  } else
    return (false);
  
  // Was able to find a suitable cell and update all vars
  return (true);
}


int ProxGrid::headingToDirection(double curr_heading) {

  int direction = -1;

  if ((curr_heading >= 0 && curr_heading <= 45) ||
      (curr_heading >= 316 && curr_heading <= 360)) {
    // xXx
    // o o
    // ooo

    direction = 0;

  } else if (curr_heading < 46 && curr_heading > 45) {
    // oxX
    // o x
    // ooo

    direction = 1;

  } else if (curr_heading >= 46 && curr_heading <= 135) {
    // oox
    // o X
    // oox

    direction = 2;

  } else if (curr_heading < 136 && curr_heading > 135) {
    // ooo
    // o x
    // oxX

    direction = 3;

  } else if (curr_heading >= 136 && curr_heading <= 225) {
    // ooo
    // o o
    // xXx

    direction = 4;

  } else if (curr_heading < 226 && curr_heading > 225) {
    // ooo
    // x o
    // Xxo

    direction = 5;

  } else if (curr_heading >= 226 && curr_heading <= 315) {
    // xoo
    // X o
    // xoo

    direction = 6;

  } else if (curr_heading < 316 && curr_heading > 315) {
    // Xxo
    // x o
    // ooo

    direction = 7;
  }

  return direction;
}


bool ProxGrid::getAheadIDs(double curr_x, double curr_y,
                                int curr_direction, int (&ahead_i)[3],
                                int (&directions)[3]) {

  double cell_size = m_grid.getCellSize();
  // int heading_vals[] = { 0, 45, 90, 135, 180, 225, 270, 315 };

  double x_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};
  double y_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};

  int xy_i[] = {0, 0, 0};

  x_vals[7] = curr_x - cell_size; //  xoo
  y_vals[7] = curr_y + cell_size; //  o o

  x_vals[0] = curr_x;               //  oxo
  y_vals[0] = curr_y + cell_size; //  o o

  x_vals[1] = curr_x + cell_size; //  oox
  y_vals[1] = curr_y + cell_size; //  o o

  x_vals[2] = curr_x + cell_size; //  ooo
  y_vals[2] = curr_y;               //  o x

  x_vals[3] = curr_x + cell_size; //  o o
  y_vals[3] = curr_y - cell_size; //  oox

  x_vals[4] = curr_x;               //  o o
  y_vals[4] = curr_y - cell_size; //  oxo

  x_vals[5] = curr_x - cell_size; //  o o
  y_vals[5] = curr_y - cell_size; //  xoo

  x_vals[6] = curr_x - cell_size; //  x o
  y_vals[6] = curr_y;               //  ooo

  if (curr_direction == 0) {
    //((curr_heading >= 0 && curr_heading <= 45) || (curr_heading >= 316 &&
    // curr_heading <= 360)) {
    // xXx
    // o o
    // ooo

    xy_i[0] = 7;
    xy_i[1] = 0;
    xy_i[2] = 1;

    directions[0] = 7;
    directions[1] = 0;
    directions[2] = 1;

  } else if (curr_direction == 1) {
    //(curr_heading < 46 && curr_heading > 45) {
    // oxX
    // o x
    // ooo

    xy_i[0] = 0;
    xy_i[1] = 1;
    xy_i[2] = 2;

    directions[0] = 0;
    directions[1] = 1;
    directions[2] = 2;

  } else if (curr_direction == 2) {
    //(curr_heading >= 46 && curr_heading <= 135) {
    // oox
    // o X
    // oox

    xy_i[0] = 1;
    xy_i[1] = 2;
    xy_i[2] = 3;

    directions[0] = 1;
    directions[1] = 2;
    directions[2] = 3;

  } else if (curr_direction == 3) {
    //(curr_heading < 136 && curr_heading > 135) {
    // ooo
    // o x
    // oxX

    xy_i[0] = 2;
    xy_i[1] = 3;
    xy_i[2] = 4;

    directions[0] = 2;
    directions[1] = 3;
    directions[2] = 4;

  } else if (curr_direction == 4) {
    //(curr_heading >= 136 && curr_heading <= 225) {
    // ooo
    // o o
    // xXx

    xy_i[0] = 3;
    xy_i[1] = 4;
    xy_i[2] = 5;

    directions[0] = 3;
    directions[1] = 4;
    directions[2] = 5;

  } else if (curr_direction == 5) {
    //(curr_heading < 226 && curr_heading > 225) {
    // ooo
    // x o
    // Xxo

    xy_i[0] = 4;
    xy_i[1] = 5;
    xy_i[2] = 6;

    directions[0] = 4;
    directions[1] = 5;
    directions[2] = 6;

  } else if (curr_direction == 6) {
    //(curr_heading >= 226 && curr_heading <= 315) {
    // xoo
    // X o
    // xoo

    xy_i[0] = 5;
    xy_i[1] = 6;
    xy_i[2] = 7;

    directions[0] = 5;
    directions[1] = 6;
    directions[2] = 7;

  } else if (curr_direction == 7) {
    //(curr_heading < 316 && curr_heading > 315) {
    // Xxo
    // x o
    // ooo

    xy_i[0] = 6;
    xy_i[1] = 7;
    xy_i[2] = 0;

    directions[0] = 6;
    directions[1] = 7;
    directions[2] = 0;

  } else {

    // wrong heading
    return false;
  }

  for (int i = 0; i < 3; i++) {

    if (!getCellID(x_vals[xy_i[i]], y_vals[xy_i[i]], ahead_i[i]))
      ahead_i[i] = -1;
  }

  return true;
}

bool ProxGrid::getBehindIDs(double curr_x, double curr_y,
                                 int curr_direction, int (&behind_i)[5]) {

  double cell_size = m_grid.getCellSize();
  double x_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};
  double y_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};

  int xy_i[] = {0, 0, 0, 0, 0};

  x_vals[7] = curr_x - cell_size; //  xoo
  y_vals[7] = curr_y + cell_size; //  o o

  x_vals[0] = curr_x;               //  oxo
  y_vals[0] = curr_y + cell_size; //  o o

  x_vals[1] = curr_x + cell_size; //  oox
  y_vals[1] = curr_y + cell_size; //  o o

  x_vals[2] = curr_x + cell_size; //  ooo
  y_vals[2] = curr_y;               //  o x

  x_vals[3] = curr_x + cell_size; //  o o
  y_vals[3] = curr_y - cell_size; //  oox

  x_vals[4] = curr_x;               //  o o
  y_vals[4] = curr_y - cell_size; //  oxo

  x_vals[5] = curr_x - cell_size; //  o o
  y_vals[5] = curr_y - cell_size; //  xoo

  x_vals[6] = curr_x - cell_size; //  x o
  y_vals[6] = curr_y;               //  ooo

  if (curr_direction == 0) {
    // xXx
    // o o
    // ooo

    xy_i[0] = 2;
    xy_i[1] = 3;
    xy_i[2] = 4;
    xy_i[3] = 5;
    xy_i[4] = 6;

  } else if (curr_direction == 1) {
    // oxX
    // o x
    // ooo

    xy_i[0] = 3;
    xy_i[1] = 4;
    xy_i[2] = 5;
    xy_i[3] = 6;
    xy_i[4] = 7;

  } else if (curr_direction == 2) {
    // oox
    // o X
    // oox

    xy_i[0] = 4;
    xy_i[1] = 5;
    xy_i[2] = 6;
    xy_i[3] = 7;
    xy_i[4] = 0;

  } else if (curr_direction == 3) {
    // ooo
    // o x
    // oxX

    xy_i[0] = 5;
    xy_i[1] = 6;
    xy_i[2] = 7;
    xy_i[3] = 0;
    xy_i[4] = 1;

  } else if (curr_direction == 4) {
    // ooo
    // o o
    // xXx

    xy_i[0] = 6;
    xy_i[1] = 7;
    xy_i[2] = 0;
    xy_i[3] = 1;
    xy_i[4] = 2;

  } else if (curr_direction == 5) {
    // ooo
    // x o
    // Xxo

    xy_i[0] = 7;
    xy_i[1] = 0;
    xy_i[2] = 1;
    xy_i[3] = 2;
    xy_i[4] = 3;

  } else if (curr_direction == 6) {
    // xoo
    // X o
    // xoo

    xy_i[0] = 0;
    xy_i[1] = 1;
    xy_i[2] = 2;
    xy_i[3] = 3;
    xy_i[4] = 4;

  } else if (curr_direction == 7) {
    // Xxo
    // x o
    // ooo

    xy_i[0] = 1;
    xy_i[1] = 2;
    xy_i[2] = 3;
    xy_i[3] = 4;
    xy_i[4] = 5;

  } else {

    // wrong heading
    return false;
  }

  for (int i = 0; i < 5; i++) {

    cout << "i " << i << " xy_i[i] " << xy_i[i] << " x_val " << x_vals[xy_i[i]]
         << " y_val " << y_vals[xy_i[i]] << endl;

    if (!getCellID(x_vals[xy_i[i]], y_vals[xy_i[i]], behind_i[i]))
      behind_i[i] = -1;
  }

  return true;
}


//---------------------------------------------------------
// Procedure: getCellData
//            Gets all the data associated with the ProxGrid

bool ProxGrid::getCellData(int idx, double &x, double &y,
                                double &val, double &var) const {
  if (idx > m_grid.size())
    return (false);

  // get x and y data
  XYSquare cell_square = m_grid.getElement(idx);
  x = cell_square.getCenterX();
  y = cell_square.getCenterY();

  if (m_grid.hasCellVar("val") and m_grid.hasCellVar("var")) {
    unsigned int cix_val = m_grid.getCellVarIX("val");
    unsigned int cix_variance = m_grid.getCellVarIX("var");
    val = m_grid.getVal(idx, cix_val);
    var = m_grid.getVal(idx, cix_variance);
  } else {
    // Not able to set value correctly
    cout << "Error reading from grid.  Did not find depth entry" << endl;
    return (false);
  }

  return (true);
}


//--------------------------------------------------------
// Procedure coolCellsVisited.
//           Cool the reward value in the cells to
//           so they approach alpha_val and alpha_var
bool ProxGrid::coolCellsVisited(double alpha_val, double alpha_var)
{
  double old_val = 0.0;
  double delta   = 0.0;
  double new_val = 0.0;
  unsigned int cix_val = 0;

  double old_var = 0.0;
  double new_var = 0.0;
  unsigned int cix_var = 0;
  
  if( m_grid.hasCellVar("val"))
    cix_val = m_grid.getCellVarIX("val");
  else
    return(false);

  if( m_grid.hasCellVar("var"))
    cix_var = m_grid.getCellVarIX("var");
  else
    return(false);
  
  for (unsigned int idx=0; idx<m_grid.size(); idx++){

    // cool the value
    old_val = m_grid.getVal(idx, cix_val);
    delta = alpha_val - old_val;
    new_val = old_val + 0.3 * delta;
    m_grid.setVal(idx, new_val, cix_val);

    // cool the variance
    old_var = m_grid.getVal(idx, cix_var);
    delta = alpha_var - old_var;
    new_var = old_var + 0.3 * delta;
    m_grid.setVal(idx, new_var, cix_var);
       
  }

  return(true);
}


//--------------------------------------------------------
// Procedure: setFrozenCell
//            

bool ProxGrid::setFrozenCell(int cell_id, double time)
{
  if (cell_id > m_grid.size())
    return (false);
  m_frozen_cells[cell_id] = time;
  return(true); 
}


//---------------------------------------------------------
// Procedure: updateFrozenCells()
bool ProxGrid::updateFrozenCells(double curr_time, double time_thresh)
{
  std::map<int, double>::iterator it;
  for (it = m_frozen_cells.begin(); it != m_frozen_cells.end();){
    double time_elapsed = curr_time - it->second;
    if ( time_elapsed <= time_thresh ) {
      ++it;
    } else {
      m_frozen_cells.erase(it++);
    }
  }
  
  return(true); 
}

//--------------------------------------------------------
// Procedure: getValueVector
//            returns vector of all values

std::vector<double> ProxGrid::getValueVector() const {
  std::vector<double> value_vector;

  if (m_grid.hasCellVar("val")) {
    unsigned int cix_depth = m_grid.getCellVarIX("val");

    unsigned int cell_count = m_grid.size();
    for (unsigned int idx = 0; idx < cell_count; idx++) {
      value_vector.push_back(m_grid.getVal(idx, cix_depth));
    }
  }

  return (value_vector);
}

//--------------------------------------------------------
// Procedure: getVarianceVector
//            returns vector of all variance values

std::vector<double> ProxGrid::getVarianceVector() const {
  std::vector<double> variance_vector;

  if (m_grid.hasCellVar("var")) {
    unsigned int cix_variance = m_grid.getCellVarIX("var");

    unsigned int cell_count = m_grid.size();
    for (unsigned int idx = 0; idx < cell_count; idx++) {
      variance_vector.push_back(m_grid.getVal(idx, cix_variance));
    }
  }

  return (variance_vector);
}
