/*****************************************************************/
/*    NAME: Tyler Paine, Nick Gershfeld                          */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: EsriBathyGrid.h                                      */
/*    DATE: Nov 2nd 2021                                         */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef ESRI_BATHY_GRID_HEADER
#define ESRI_BATHY_GRID_HEADER

#include <string>
#include <vector>
#include <map>
#include <iostream>  // for cout
#include <fstream>   // for ifstream to read and write file
#include <cmath>     // std::floor
#include <queue>

#include "XYConvexGrid.h" 
#include "XYPolygon.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"  // lat/lon conv
#include "XYSquare.h"
#include "XYGridUpdate.h"
#include "XYFormatUtilsSegl.h"



class EsriBathyGrid
{
public:
  EsriBathyGrid();
  EsriBathyGrid(XYConvexGrid new_grid, double y_origin_lat, double x_origin_lon, double no_data_value);
  ~EsriBathyGrid() {}

  
  bool loadGridFromFile(std::string filename, double x_origin_lon, double y_origin_lat);
  bool saveGridToFile(std::string filename);

  bool updateCellValueXY(double x, double y, std::vector<std::string> cell_vars, std::vector<double> cell_vals);
  bool updateCellValueIDX(unsigned int idx, std::vector<std::string> cell_vars, std::vector<double> cell_vals);

  // Getters
  bool getCellData(unsigned int idx, double &x, double &y, double &depth, double &var) const;       // given id; get x, y, depth, variance 
  bool getCellXY(unsigned int idx, double &x, double &y) const;                                     // given id; get x, y
  bool getCellDataXY(double x, double y, double &depth, double &var) const;                         // given xy; get depth, variance
  bool getCellDataXYI(double x, double y, double &depth, double &var, double &i_out) const;         // given xy; get id, depth, variance
  bool getCellID(double x, double y, int &cell_id) const;                                           // given xy; get id
  bool getCellCenter(double x, double y, int &cell_id, double &center_x, double &center_y) const;   // given xy; get id, center xy

  unsigned long int getNCols() {calculateRowsCols(); return(m_n_cols);}
  unsigned long int getNRows() {calculateRowsCols(); return(m_n_rows);}

  std::map<size_t, std::vector<double> > getVertices() {return m_V;}
  std::vector<std::pair<size_t,size_t> > getEdges() {return m_E;}
  std::set<size_t> getObstacles(double depth_threshold, double var_threshold);
  std::set<size_t> getFinalObstacles(double depth_threshold, double var_threshold);
  std::string getObstacleGridSpec(double depth_threshold, double var_threshold);

  double getPathLength(std::vector<size_t> const &path) const;

  double getCellSize()  {return m_cell_size;}
  double getMaxVar();
  double getGridAverageDepth();
  
  std::vector<double> getDepthVector() const;
  std::vector<double> getVarianceVector() const;
  std::string getDepthVectorString();
  std::string getVarianceVectorString();
  std::string getPathVectorString();

  std::string getXYString();
  
  bool getBounds(double &min_x, double &min_y, double &max_x, double &max_y);
  bool getNextCell(double curr_x, double curr_y, double curr_heading, double &new_x, double &new_y, int &new_id);
  bool getAheadIDs(double curr_x, double curr_y, int curr_direction, int (&ahead_i)[3], int (&directions)[3]);
  bool getBehindIDs(double curr_x, double curr_y, int curr_direction, int (&behind_i)[5]);

  int headingToDirection(double curr_heading);
  
  std::string getGridSpec() const { return(m_grid.get_spec());}
  std::string getVarianceGridSpec();

  XYSegList getBorderSegList();
  std::set<unsigned int> getCellsTop();
  std::set<unsigned int> getCellsBottom();
  std::set<unsigned int> getCellsBetween(double x1, double y1, double x2, double y2);
  

  // Grid delta support
  std::string getDeltaSpec(double tolerance, bool &found_delta); 
 
  
  // Setters
  void setNCols(unsigned long int ncols)  {m_n_cols = ncols; return;}
  void setNRows(unsigned long int nrows)  {m_n_rows = nrows;}
  
  void setXllCorner(double x_crnr)  {m_x_ll_corner = x_crnr; return;}
  void setYllCorner(double y_crnr)  {m_y_ll_corner = y_crnr; return;}
  void setCellSize(double cell_size)  {m_cell_size = cell_size; return;}
  void setNoDataValue(double no_val)  {m_no_data_value = no_val; return;}
  void setDefaultVarianceValue(double d_var) {m_default_variance = d_var; return;}
  void setXYConvexGrid(XYConvexGrid new_grid);
  
  bool setXYOrigin(double x_origin_lon, double y_origin_lat);
 
  unsigned int size() const {return(m_grid.size());}

  static std::string convertToConvexGrid(std::string poly_spec);
  
  // Grid delta support 
  void setOldGridToNew(); // sets the old grid to the new grid.  No more delta.
  bool processGridDelta(std::string str) { return(m_grid.processDelta(str));}

 private:
  bool calculateRowsCols();
  bool readEsriLine(std::string lineIn, double &lat, double &lon, double &depth);

  void generateEdges();
  void generateVertices();
  
  //int get_max_index(double * array, int size);
  XYConvexGrid m_grid;
  XYConvexGrid m_old_grid;
  unsigned long int m_n_cols;
  unsigned long int m_n_rows;
  
  double m_x_ll_corner;
  double m_y_ll_corner;
  double m_cell_size;
  double m_default_variance;

  double m_x_origin_lon;
  double m_y_origin_lat;
  double m_no_data_value;

  CMOOSGeodesy m_geodesy;

  std::map<size_t, std::vector<double> > m_V;
  std::vector<std::pair<size_t,size_t> > m_E;
  std::map<std::vector<double>, size_t> m_reverseV;
 
};






#endif
