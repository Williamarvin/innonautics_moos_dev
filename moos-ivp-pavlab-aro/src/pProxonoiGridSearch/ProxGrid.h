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

#ifndef PROX_GRID_HEADER
#define PROX_GRID_HEADER

#include <vector>
#include <map>
#include "XYConvexGrid.h"
#include "XYPolygon.h"
#include "XYSquare.h"


class ProxGrid
{
public:
  ProxGrid();
  ProxGrid(XYConvexGrid new_grid, double no_data_value);
  ~ProxGrid() {}

  bool getCellID(double x, double y, int &cell_id);       // given xy; get id
  bool getCellXY(int idx, double &x, double &y);	// given id; get x, y
  bool updateCellValueIDX(int idx,
			  double value, double variance);

  int  headingToDirection(double curr_heading);
  
  bool getAheadIDs(double curr_x, double curr_y,
                   int curr_direction, int (&ahead_i)[3],
		   int (&directions)[3]);
  
  bool getBehindIDs(double curr_x, double curr_y,
		    int curr_direction, int (&behind_i)[5]);

  bool getCellData(int idx, double &x, double &y,
		   double &val, double &var) const;

  std::vector<double> getValueVector() const;
  std::vector<double> getVarianceVector() const;
  
  bool coolCellsVisited(double alpha_val, double alpha_var);
  bool setFrozenCell(int cell_id, double time);
  bool updateFrozenCells(double curr_time, double time_thresh);
  
  std::string get_spec() const {return(m_grid.get_spec());}
  double getCellSize() const {return(m_grid.getCellSize());}
  
 private:
  XYConvexGrid m_grid;
  double m_no_data_value;

  // keyed on cell index, double is time
  std::map<int, double> m_frozen_cells;

};


#endif
