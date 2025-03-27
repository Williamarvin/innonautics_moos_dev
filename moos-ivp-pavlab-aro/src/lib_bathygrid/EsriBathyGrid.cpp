/*****************************************************************/
/*    NAME: Tyler Paine, Nick Gershfeld                          */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: EsriBathyGrid.cpp                                    */
/*    DATE: Nov 2nd 2021                                         */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include "EsriBathyGrid.h"

using namespace std;

//---------------------------------------------------------
// Constructor
EsriBathyGrid::EsriBathyGrid() {

  m_n_cols = 0;
  m_n_cols = 0;

  m_cell_size = 0.0;
  m_x_ll_corner = 0.0;
  m_y_ll_corner = 0.0;
  m_no_data_value = 0.0;
  m_default_variance = 0.0;

  generateVertices();
  generateEdges();
}

//---------------------------------------------------------
// Constructor
EsriBathyGrid::EsriBathyGrid(XYConvexGrid new_grid, double y_origin_lat,
                             double x_origin_lon, double no_data_value) {

  m_n_cols = 0;
  m_n_rows = 0;

  m_cell_size = 0.0;
  m_x_ll_corner = 0.0;
  m_y_ll_corner = 0.0;
  m_default_variance = 0.0;

  m_x_origin_lon = x_origin_lon;
  m_y_origin_lat = y_origin_lat;
  m_no_data_value = no_data_value;

  m_grid = new_grid;
  m_old_grid = new_grid;
  m_cell_size = m_grid.getCellSize();

  generateVertices();
  generateEdges();
}

//---------------------------------------------------------
// Procedure: readGridFromFile
//            Loads a bathymetry grid from Esri-formatted file
//            Loads all the depth data as the inital values

bool EsriBathyGrid::loadGridFromFile(std::string filename, double x_origin_lon,
                                     double y_origin_lat) {
  /////////////////////////////////////////////////////////////
  // Step 1.  Load the meta data from the file XXXXXX.asc

  // Example file:
  // ncols  (\t)	1307
  // nrows  (\t)	1180
  // xllcorner (\t)	-79.94965
  // yllcorner (\t)	32.71581
  // cellsize  (\t)	10
  // nodata_value (\t)	-99999

  string meta_filename = filename + ".asc";
  std::ifstream infile(meta_filename);

  std::string line;
  while (std::getline(infile, line)) {
    // Parse string by biting
    string param;
    bool handled = false;

    param = biteStringX(line, '\t');

    if (param == "ncols") {
      handled = isNumber(line);
      if (handled) {
        m_n_cols = stoul(line, nullptr, 10);
      }

    } else if (param == "nrows") {
      handled = isNumber(line);
      if (handled) {
        m_n_rows = stoul(line, nullptr, 10);
      }
    } else if (param == "xllcorner") {
      handled = setDoubleOnString(m_x_ll_corner, line);

    } else if (param == "yllcorner") {
      handled = setDoubleOnString(m_y_ll_corner, line);

    } else if (param == "cellsize") {
      handled = setDoubleOnString(m_cell_size, line);

    } else if (param == "nodata_value") {
      handled = setDoubleOnString(m_no_data_value, line);

    } else if (isNumber(param)) {
      // This is a raster value, ignore it.
      handled = true;
    }

    if (!handled)
      cout << "Error: Unhandled Esri-Grid meta parameter: " << param << endl;
  }

  // Check if we have valid cols, rows and cell size data
  if ((m_n_cols == 0) or (m_n_rows == 0) or (m_cell_size)) {
    cout << "Error: Did not find the number of rows or colums, nor the cell "
            "size."
         << endl;
    cout << "Check the meta data file: " << meta_filename << endl;
    return (false);
  }

  // set the XY origin for Geodesy
  bool geoOK = setXYOrigin(x_origin_lon, y_origin_lat);
  if (!geoOK)
    return (false);

  ////////////////////////////////////////////////////////////////
  // Step 2.  Load the grid data from the file XXXXXX.xyz

  // Example file format:
  // -122.40291  (\t)	37.82767  (\t)	-22.00000
  //     :        :         :      :         :
  //     :        :         :      :         :

  // load it into the grid

  //                    ******* Grid layout *******
  //____________________________________________________________________
  //
  //                             Longitude
  //  <- more negative values (west)     less negative numbers(east) ->
  //
  //______________________________Column________________________________
  //  0, 1, 2, ...                                          ... m_n_cols
  //
  //

  // Row        ^
  //  0         |
  //  1         more positive (north)
  //  2                                   Latitude
  //  :         less positive (south)
  //  :         |
  //  m_n_rows  v

  // (the ll corner is the most negative lat and long)

  // Step 2a: first find the bounds of the outer polygon grid
  //          and initialize with the no_valid_data number

  string data_filename = filename + ".xyz";
  std::ifstream infile2a(data_filename);

  std::string line2;
  double max_lat = -180.0;
  double max_lon = -90.0;
  double min_lat = 90.0;
  double min_lon = 180.0;

  double lon;
  double lat;
  double depth;

  // Open the file and keep a running tab of the min max values.
  // This was done here and repeated below to save memory, but is
  // slower.
  while (std::getline(infile2a, line2)) {
    readEsriLine(line2, lat, lon, depth);
    if (lat > max_lat)
      max_lat = lat;
    if (lat < min_lat)
      min_lat = lat;
    if (lon > max_lon)
      max_lon = lon;
    if (lon < min_lon)
      min_lon = lon;
  }
  //  Convert from lat long to x y.
  double min_x, max_x, min_y, max_y;
  bool ok1 = m_geodesy.LatLong2LocalGrid(min_lat, min_lon, min_y, min_x);
  bool ok2 = m_geodesy.LatLong2LocalGrid(max_lat, max_lon, max_y, max_x);
  if (!ok1 or !ok2) {
    cout << "Error: Geodesy did not convert from lat long to x, y" << endl;
  }

  // Each lat/lon entry in the ESRI data file is the center of grid square,
  // so need to add a border that is half of the cell size
  double xlow = min_x - m_cell_size / 2;
  double xhigh = max_x + m_cell_size / 2;
  double ylow = min_y - m_cell_size / 2;
  double yhigh = max_y + m_cell_size / 2;

  // Create a bounding box around the entire grid
  XYPolygon esri_grid_poly;
  esri_grid_poly.add_vertex(xlow, ylow, false);
  esri_grid_poly.add_vertex(xhigh, ylow, false);
  esri_grid_poly.add_vertex(xhigh, yhigh, false);
  esri_grid_poly.add_vertex(xlow, yhigh, true);

  // Set the cell depth and variance to "no valid data" and default values
  vector<string> cell_vars;
  cell_vars.push_back("depth");
  cell_vars.push_back("var");
  vector<double> cell_vals;
  cell_vals.push_back(m_no_data_value);
  cell_vals.push_back(m_default_variance);

  // Finally, initialize the grid.
  bool ok3;
  ok3 = m_grid.initialize(esri_grid_poly, m_cell_size, cell_vars, cell_vals);
  if (!ok3) {
    cout << "Error: Failure to initialze the EsriGrid" << endl;
  }

  // Step 2b. Now load all the values from the file into the grid

  std::ifstream infile2b(data_filename);
  while (std::getline(infile2b, line2)) {
    readEsriLine(line2, lat, lon, depth);

    double x, y;
    bool ok4 = m_geodesy.LatLong2LocalGrid(lat, lon, y, x);
    if (!ok4) {
      cout << "Error: Geodesy did not convert from lat long to x, y" << endl;
    }

    // Set the cell depth
    vector<string> cell_vars;
    cell_vars.push_back("depth");
    cell_vars.push_back("var");
    vector<double> cell_vals;
    cell_vals.push_back(depth);
    cell_vals.push_back(m_default_variance);

    // Add the cell to the grid
    bool ok5;
    ok5 = updateCellValueXY(x, y, cell_vars, cell_vals);
    if (!ok5) {
      cout << "Error: Failure to update grid at x = " << x << " y = " << y
           << endl;
    }
  }

  return true;
}

//---------------------------------------------------------
// Procedure: generateVertices
//            modifies m_V to contain a map of ids to xy
//            modifies m_reverseV to contain a map of xy to ids
void EsriBathyGrid::generateVertices() {

  unsigned int cell_count = m_grid.size();
  for (int idx = 0; idx < cell_count; idx++) {

    // get x and y data
    XYSquare cell_square = m_grid.getElement(idx);
    double x = cell_square.getCenterX();
    double y = cell_square.getCenterY();

    vector<double> coords;
    coords.push_back(x);
    coords.push_back(y);

    m_V[idx] = coords;
    m_reverseV[coords] = idx;
  }
}

//---------------------------------------------------------
// Procedure: generateEdges
//            modifies m_E to include all connections between grid cells
//            m_E takes the form of a vector of index pairs
void EsriBathyGrid::generateEdges() {

  for (int idx = 0; idx < m_grid.size(); idx++) {

    // get neighbors
    vector<double> curr_coords = m_V.at(idx);
    double curr_x = curr_coords[0];
    double curr_y = curr_coords[1];

    double x_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};
    double y_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};
    int i_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};

    x_vals[7] = curr_x - m_cell_size; //  xoo
    y_vals[7] = curr_y + m_cell_size; //  o o

    x_vals[0] = curr_x;               //  oxo
    y_vals[0] = curr_y + m_cell_size; //  o o

    x_vals[1] = curr_x + m_cell_size; //  oox
    y_vals[1] = curr_y + m_cell_size; //  o o

    x_vals[2] = curr_x + m_cell_size; //  ooo
    y_vals[2] = curr_y;               //  o x

    x_vals[3] = curr_x + m_cell_size; //  o o
    y_vals[3] = curr_y - m_cell_size; //  oox

    x_vals[4] = curr_x;               //  o o
    y_vals[4] = curr_y - m_cell_size; //  oxo

    x_vals[5] = curr_x - m_cell_size; //  o o
    y_vals[5] = curr_y - m_cell_size; //  xoo

    x_vals[6] = curr_x - m_cell_size; //  x o
    y_vals[6] = curr_y;               //  ooo

    for (int i = 0; i < 8; i++) {

      // convert to index
      vector<double> tmp_coords;
      tmp_coords.push_back(x_vals[i]);
      tmp_coords.push_back(y_vals[i]);

      if (m_reverseV.count(tmp_coords)) {
        i_vals[i] = m_reverseV.at(tmp_coords);

        // if not in edges, add edge
        pair<size_t, size_t> pair_a(idx, i_vals[i]);
        pair<size_t, size_t> pair_b(i_vals[i], idx);

        bool in_vector = false;

        for (vector<pair<size_t, size_t> >::reverse_iterator p = m_E.rbegin();
             p != m_E.rend(); ++p) {
          if (*p == pair_a || *p == pair_b)
            in_vector = true;
        }

        if (!in_vector) {
          m_E.push_back(pair_a);
        }

      } else {
        i_vals[i] = -1;
      }
    }
  }
}

//---------------------------------------------------------
// Procedure: getObstacles
//            Returns a set of indices that are too shallow
//            and we are confident that it is too shallow
set<size_t> EsriBathyGrid::getObstacles(double depth_threshold,
                                        double var_threshold) {
  double tmp_depth;
  double tmp_var;
  set<size_t> obstacles;

  if (m_grid.hasCellVar("depth")) {

    // cout << "getting obstacles" << endl;

    unsigned int cix_depth = m_grid.getCellVarIX("depth");
    unsigned int cix_var = m_grid.getCellVarIX("var");
    unsigned int cell_count = m_grid.size();

    for (int idx = 0; idx < cell_count; idx++) {
      tmp_depth = m_grid.getVal(idx, cix_depth);
      tmp_var = m_grid.getVal(idx, cix_var);

      bool shallow = (tmp_depth < depth_threshold && tmp_var < var_threshold);

      if (shallow) { // confident that it's shallow
        obstacles.insert(idx);
      }
    }
  }

  return obstacles;
}

//---------------------------------------------------------
// Procedure: getObstacleGridSpec
//            Returns a grid spec showing obstacles for the path proposal in
//            hybrid mode 1 where no obstacle and 0 where obstacle
string EsriBathyGrid::getObstacleGridSpec(double depth_threshold,
                                          double var_threshold) {

  // get obstacles
  set<size_t> obs = getObstacles(depth_threshold, var_threshold);

  // BUILD CONFIG SPEC

  string tmpspc = m_grid.getConfigStr();

  int border_end = tmpspc.find("}") + 1;
  string spec = tmpspc.substr(0, border_end);

  spec += ",cell_size=" + to_string((int)m_cell_size) + ",";
  spec += "cell_vars=x:0,cell_min=x:0,cell_max=x:1";

  // BUILD ELEMENTS SPEC

  // iterate through obstacles and add to spec
  unsigned int obs_count = obs.size();
  for (set<size_t>::iterator itr = obs.begin(); itr != obs.end(); itr++) {

    string cell_spec = ",cell=" + uintToString(*itr) + ":x:1";
    spec += cell_spec;
  }

  spec += ",label=obs";

  return (spec);
}

//---------------------------------------------------------
// Procedure: getFinalObstacles
//            Returns a set of indices that do not meet the depth threshold
set<size_t> EsriBathyGrid::getFinalObstacles(double depth_threshold,
                                             double var_threshold) {
  double tmp_depth;
  double tmp_var;
  set<size_t> obstacles;

  if (m_grid.hasCellVar("depth")) {

    // cout << "getting obstacles" << endl;

    unsigned int cix_depth = m_grid.getCellVarIX("depth");
    unsigned int cix_var = m_grid.getCellVarIX("var");
    unsigned int cell_count = m_grid.size();

    for (int idx = 0; idx < cell_count; idx++) {
      tmp_depth = m_grid.getVal(idx, cix_depth);

      bool shallow = (tmp_depth < depth_threshold);

      // cout << "id " << idx << " depth " << tmp_depth << " obs " << shallow <<
      // endl;

      if (shallow) { // it's shallow
        obstacles.insert(idx);
      }
    }
  }

  return obstacles;
}

//---------------------------------------------------------
// Procedure: saveGridToFile
//            Saves a bathymetry grid to Esri-formatted file
//            No checks!!!  Assumes a completely populated
//            xy grid is present where unknown depth values
//            are ok.

bool EsriBathyGrid::saveGridToFile(string filename) {

  // Step 1.  Save out the xyz data
  // Example file format:
  // -122.40291  (\t)	37.82767  (\t)	-22.00000
  //     :        :         :      :         :
  //     :        :         :      :         :

  string xyz_filename = filename + ".xyz";

  // Initialize CMOOSGeodesy object - (again? if necessary)
  //                                  (I don't see why not)
  bool geoOK = m_geodesy.Initialise(m_y_origin_lat, m_x_origin_lon);
  if (!geoOK) {
    cout << "CMOOSGeodesy::Initialise() failed. Invalid origin. " << endl;
    cout << "Failed when writing out the .xyz file, check values in MOOS. "
         << endl;
    return (false);
  }

  ofstream xyz_file(xyz_filename);
  if (xyz_file.is_open()) {

    // Write out the information from each cell
    unsigned int cell_count = m_grid.size();
    unsigned int cix;
    double cell_depth;

    XYSquare cell_square;
    double x_center;
    double y_center;
    double lat_out;
    double lon_out;

    double min_lat = 90.0;
    double min_lon = 180.0;

    for (int idx = 0; idx < cell_count; idx++) {

      if (m_grid.hasCellVar("depth")) {
        cix = m_grid.getCellVarIX("depth");
        cell_depth = m_grid.getVal(idx, cix);
        cell_square = m_grid.getElement(idx);
        x_center = cell_square.getCenterX();
        y_center = cell_square.getCenterY();

        // transform back in long/lat.
        bool transform_ok;
        transform_ok =
            m_geodesy.LocalGrid2LatLong(x_center, y_center, lat_out, lon_out);

        // update minimum values
        if (lat_out < min_lat)
          min_lat = lat_out;
        if (lon_out < min_lon)
          min_lon = lon_out;

        // write out
        if (transform_ok) {
          string str_lon = doubleToString(lon_out, 5);
          string str_lat = doubleToString(lat_out, 5);
          string str_depth = doubleToString(cell_depth, 5);
          xyz_file << str_lon << '\t' << str_lat << '\t' << str_depth << '\n';
        }
      }
    }
    xyz_file.close();

    // Update the lower left corners
    m_x_ll_corner = min_lon;
    m_y_ll_corner = min_lat;

  } else {
    cout << "Error:  Unable to open xyz file to write." << endl;
    return (false);
  }

  // Step 2.  Save out the meta_data
  // Example file:
  // ncols  (\t)	1307
  // nrows  (\t)	1180
  // xllcorner (\t)	-79.94965
  // yllcorner (\t)	32.71581
  // cellsize  (\t)	10
  // nodata_value (\t)	-99999

  // update number of rows and colums.
  calculateRowsCols();

  string asc_filename = filename + ".asc";

  ofstream asc_file(asc_filename);
  if (asc_file.is_open()) {
    asc_file << "ncols" << '\t' << ulintToString(m_n_cols) << '\n';
    asc_file << "nrows" << '\t' << ulintToString(m_n_rows) << '\n';
    asc_file << "xllcorner" << '\t' << doubleToString(m_x_ll_corner, 5) << '\n';
    asc_file << "yllcorner" << '\t' << doubleToString(m_y_ll_corner, 5) << '\n';
    asc_file << "cellsize" << '\t' << doubleToString(m_cell_size, 5) << '\n';
    asc_file << "nodata_value" << '\t' << doubleToString(m_no_data_value, 5)
             << '\n';

    unsigned int cell_count = m_grid.size();
    unsigned int cix;
    double cell_depth;

    for (int idx = 0; idx < cell_count; idx++) {

      if (m_grid.hasCellVar("depth")) {
        cix = m_grid.getCellVarIX("depth");
        cell_depth = m_grid.getVal(idx, cix);

        // write out
        string str_depth = doubleToString(cell_depth, 5);
        asc_file << str_depth << " ";
      }

      if ((idx + 1) % m_n_cols == 0) { // new row
        asc_file << '\n';
      }
    }

    asc_file.close();
  } else {
    cout << "Error:  Unable to open asc file to write." << endl;
    return (false);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: updateCellValueXY
//            Updates the cell that contains this x,y position
//            with the new values
//            This is a convenience function, as it just uses other
//            functions in the class.
//            Returns false if no cell was found that contains the
//            x y position

bool EsriBathyGrid::updateCellValueXY(double x, double y,
                                      vector<string> cell_vars,
                                      vector<double> cell_vals) {

  if (cell_vars.size() != cell_vals.size())
    return (false);

  XYSquare perimeter = m_grid.getSBound();
  if (!perimeter.containsPoint(x, y))
    return (false);

  // find the appropriate cell.
  unsigned int cell_count = m_grid.size();
  for (int idx = 0; idx < cell_count; idx++) {

    bool contains_point = m_grid.ptIntersect(idx, x, y);
    if (contains_point) {

      bool update_ok = updateCellValueIDX(idx, cell_vars, cell_vals);
      return (update_ok);
    }
  }
  // Was NOT able to find a suitable cell for this location
  return (false);
}

//---------------------------------------------------------
// Procedure: updateCellValueIDX
//            Updates the cell based on index
//            with the new value - in this case depth
//            This is a convenience function, as it just uses other
//            functions in the XYConvexGrid class.

bool EsriBathyGrid::updateCellValueIDX(unsigned int idx,
                                       vector<string> cell_vars,
                                       vector<double> cell_vals) {
  if (cell_vars.size() != cell_vals.size())
    return (false);

  // iterate through the cell vars to update
  unsigned int j, vsize = cell_vars.size();
  for (j = 0; j < vsize; j++) {
    if (m_grid.hasCellVar(cell_vars[j])) {
      unsigned int cix = m_grid.getCellVarIX(cell_vars[j]);
      m_grid.setVal(idx, cell_vals[j], cix);
    } else {
      // Not able to set value correctly
      cout << "Error updating grid.  Following cell value not found: "
           << cell_vars[j] << endl;
      return (false);
    }
  }

  // Was able to find a suitable cell and update all vars
  return (true);
}

//---------------------------------------------------------
// Procedure: readEsriLine
//            parses and updates the lat, lon, and depth
//            value from an ascii line in the Esri-Grid file

bool EsriBathyGrid::readEsriLine(string lineIn, double &lat, double &lon,
                                 double &depth) {

  double temp_lat;
  double temp_lon;
  double temp_depth;

  string first = biteStringX(lineIn, '\t');
  string second = biteStringX(lineIn, '\t');
  string remainder = lineIn;

  bool ok1, ok2, ok3;
  if (isNumber(first) and isNumber(second) and isNumber(remainder)) {
    ok1 = setDoubleOnString(temp_lon, first);
    ok2 = setDoubleOnString(temp_lat, second);
    ok3 = setDoubleOnString(temp_depth, remainder);
  }
  if (!ok1 and !ok2 and !ok3) {
    cout << "Error reading line Esri-Grid file: " << endl;
    cout << "Cannot read line: Long = " << first << " Lat = " << second
         << " Depth = " << remainder << endl;
    return (false);
  }

  // check for reasonable lat and long values
  if (abs(temp_lat) >= 90.0)
    return (false);
  if (abs(temp_lon) > -180.0)
    return (false);

  lat = temp_lat;
  lon = temp_lon;
  depth = temp_depth;
  return (true);
}

//---------------------------------------------------------
// Procedure: getCellData
//            Gets all the data associated with the EsriGrid

bool EsriBathyGrid::getCellData(unsigned int idx, double &x, double &y,
                                double &depth, double &var) const {
  if (idx > m_grid.size())
    return (false);

  // get x and y data
  XYSquare cell_square = m_grid.getElement(idx);
  x = cell_square.getCenterX();
  y = cell_square.getCenterY();

  if (m_grid.hasCellVar("depth") and m_grid.hasCellVar("var")) {
    unsigned int cix_depth = m_grid.getCellVarIX("depth");
    unsigned int cix_variance = m_grid.getCellVarIX("var");
    depth = m_grid.getVal(idx, cix_depth);
    var = m_grid.getVal(idx, cix_variance);
  } else {
    // Not able to set value correctly
    cout << "Error reading from grid.  Did not find depth entry" << endl;
    return (false);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: getCellXY
//            Gets the xy values of a cell id

bool EsriBathyGrid::getCellXY(unsigned int idx, double &x, double &y) const {
  if (idx > m_grid.size())
    return (false);

  // get x and y data
  XYSquare cell_square = m_grid.getElement(idx);
  x = cell_square.getCenterX();
  y = cell_square.getCenterY();

  return (true);
}

bool EsriBathyGrid::getCellDataXY(double x, double y, double &depth,
                                  double &var) const {

  XYSquare perimeter = m_grid.getSBound();
  if (!perimeter.containsPoint(x, y))
    return (false);

  // find the appropriate cell.
  unsigned int cell_count = m_grid.size();
  for (int idx = 0; idx < cell_count; idx++) {

    bool contains_point = m_grid.ptIntersect(idx, x, y);
    if (contains_point) {

      if (m_grid.hasCellVar("depth") and m_grid.hasCellVar("var")) {
        unsigned int cix_depth = m_grid.getCellVarIX("depth");
        unsigned int cix_variance = m_grid.getCellVarIX("var");
        depth = m_grid.getVal(idx, cix_depth);
        var = m_grid.getVal(idx, cix_variance);
      } else {
        // Not able to set value correctly
        cout << "Error reading from grid.  Did not find depth entry" << endl;
        return (false);
      }

      return (true);
    }
  }
  // Was NOT able to find a suitable cell for this location
  return (false);
}

bool EsriBathyGrid::getCellDataXYI(double x, double y, double &depth,
                                   double &var, double &i_out) const {

  XYSquare perimeter = m_grid.getSBound();
  if (!perimeter.containsPoint(x, y))
    return (false);

  // find the appropriate cell.
  unsigned int cell_count = m_grid.size();
  for (int idx = 0; idx < cell_count; idx++) {

    bool contains_point = m_grid.ptIntersect(idx, x, y);
    if (contains_point) {

      if (m_grid.hasCellVar("depth") and m_grid.hasCellVar("var")) {
        unsigned int cix_depth = m_grid.getCellVarIX("depth");
        unsigned int cix_variance = m_grid.getCellVarIX("var");
        depth = m_grid.getVal(idx, cix_depth);
        var = m_grid.getVal(idx, cix_variance);
        i_out = idx;
      } else {
        // Not able to set value correctly
        cout << "Error reading from grid.  Did not find depth entry" << endl;
        return (false);
      }

      return (true);
    }
  }
  // Was NOT able to find a suitable cell for this location
  return (false);
}

bool EsriBathyGrid::getCellID(double x, double y, int &cell_id) const {

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

  cout << "Did not find " << x << "," << y << endl;
  return (false);
}

bool EsriBathyGrid::getCellCenter(double x, double y, int &cell_id,
                                  double &center_x, double &center_y) const {

  XYSquare perimeter = m_grid.getSBound();
  if (!perimeter.containsPoint(x, y)) {
    cout << "out of bounds " << x << "," << y << endl;
    return (false);
  }

  // find the appropriate cell.
  unsigned int cell_count = m_grid.size();
  for (int idx = 0; idx < cell_count; idx++) {

    bool contains_point = m_grid.ptIntersect(idx, x, y);
    if (contains_point) {
      cell_id = idx;

      XYSquare cell_square = m_grid.getElement(idx);
      center_x = cell_square.getCenterX();
      center_y = cell_square.getCenterY();
      return (true);
    }
  }
  // Was NOT able to find a suitable cell for this location

  cout << "did not find " << x << "," << y << endl;
  return (false);
}

//--------------------------------------------------------
// Procedure: getDepthVector
//            returns vector of all depth values

vector<double> EsriBathyGrid::getDepthVector() const {
  vector<double> depth_vector;

  if (m_grid.hasCellVar("depth")) {
    unsigned int cix_depth = m_grid.getCellVarIX("depth");

    unsigned int cell_count = m_grid.size();
    for (unsigned int idx = 0; idx < cell_count; idx++) {
      depth_vector.push_back(m_grid.getVal(idx, cix_depth));
    }
  }

  return depth_vector;
}

//--------------------------------------------------------
// Procedure: getVarianceVector
//            returns vector of all variance values

vector<double> EsriBathyGrid::getVarianceVector() const {
  vector<double> variance_vector;

  if (m_grid.hasCellVar("var")) {
    unsigned int cix_variance = m_grid.getCellVarIX("var");

    unsigned int cell_count = m_grid.size();
    for (unsigned int idx = 0; idx < cell_count; idx++) {
      variance_vector.push_back(m_grid.getVal(idx, cix_variance));
    }
  }

  return variance_vector;
}

//--------------------------------------------------------
// Procedure: getDepthVectorString
//            returns comma separated string of all depth values

string EsriBathyGrid::getDepthVectorString() {
  string depth_string = "";

  if (m_grid.hasCellVar("depth")) {
    unsigned int cix_depth = m_grid.getCellVarIX("depth");

    unsigned int cell_count = m_grid.size();
    for (int idx = 0; idx < cell_count; idx++) {
      depth_string += to_string(m_grid.getVal(idx, cix_depth));

      if (idx != (cell_count - 1))
        depth_string += ",";
    }
  }

  return depth_string;
}

//--------------------------------------------------------
// Procedure: getVarianceVectorString
//            returns comma separated string of all variance values

string EsriBathyGrid::getVarianceVectorString() {
  string variance_string = "";

  if (m_grid.hasCellVar("var")) {
    unsigned int cix_variance = m_grid.getCellVarIX("var");

    unsigned int cell_count = m_grid.size();
    for (int idx = 0; idx < cell_count; idx++) {
      variance_string += to_string(m_grid.getVal(idx, cix_variance));

      if (idx != (cell_count - 1))
        variance_string += ",";
    }
  }

  return variance_string;
}

//--------------------------------------------------------
// Procedure: getPathVectorString
//            returns comma separated string of all path values from a psg

string EsriBathyGrid::getPathVectorString() {
  string path_string = "";

  if (m_grid.hasCellVar("x")) {
    unsigned int cix_path = m_grid.getCellVarIX("x");

    unsigned int cell_count = m_grid.size();
    for (int idx = 0; idx < cell_count; idx++) {
      path_string += to_string(m_grid.getVal(idx, cix_path));

      if (idx != (cell_count - 1))
        path_string += ",";
    }
  }

  return path_string;
}

//--------------------------------------------------------
// Procedure: getXYString
//            returns a string of the x,y values of each cell
string EsriBathyGrid::getXYString() {
  string xy_str = "";

  unsigned int cell_count = m_grid.size();
  for (int idx = 0; idx < cell_count; idx++) {
    XYSquare cell_square = m_grid.getElement(idx);
    double x = cell_square.getCenterX();
    double y = cell_square.getCenterY();

    xy_str += to_string(x) + " " + to_string(y);

    if (idx < cell_count - 1)
      xy_str += " ";
  }

  return xy_str;
}

//--------------------------------------------------------
// Procedure: getMaxVariance
//            returns maximum variance value

double EsriBathyGrid::getMaxVar() {
  double max_var = m_default_variance;

  if (m_grid.hasCellVar("var")) {
    unsigned int cix_variance = m_grid.getCellVarIX("var");

    unsigned int cell_count = m_grid.size();
    for (int idx = 0; idx < cell_count; idx++) {
      double tmp_var = m_grid.getVal(idx, cix_variance);
      if (tmp_var > max_var)
        max_var = tmp_var;
    }
  }

  return max_var;
}

//--------------------------------------------------------
// Procedure: setXYOrigin
//            initializes Geodesy with XY

bool EsriBathyGrid::setXYOrigin(double x_origin_lon, double y_origin_lat) {
  // Initialize CMOOSGeodesy object
  bool geoOK = m_geodesy.Initialise(y_origin_lat, x_origin_lon);
  if (!geoOK) {
    cout << "CMOOSGeodesy::Initialise() failed. Invalid origin. " << endl;
    cout << "Check the xllcorner and yllcorner values in the metadata file: "
         << endl;
  }

  m_x_origin_lon = x_origin_lon;
  m_y_origin_lat = y_origin_lat;

  return (geoOK);
}

//------------------------------------------------------
// Procedure:  calculate rows and columns
//
bool EsriBathyGrid::calculateRowsCols() {
  // Get outer bounding polygon
  XYSquare outer_square = m_grid.getSBound();
  // Get cell size used to initialize
  //  (again just in case)
  m_cell_size = m_grid.getCellSize();

  // Don't divide by zero.
  if (m_cell_size == 0.0)
    return (false);

  double y_length = outer_square.getLengthY();
  double x_length = outer_square.getLengthX();

  m_n_rows = (unsigned long int)std::ceil(y_length / m_cell_size);
  m_n_cols = (unsigned long int)std::ceil(x_length / m_cell_size);

  return (true);
}

//---------------------------------------------------------
//  Procedure setXYConvexGrid()
void EsriBathyGrid::setXYConvexGrid(XYConvexGrid new_grid) {
  if (m_grid.size() > 0)
    m_old_grid = m_grid;
  else
    m_old_grid = new_grid;

  m_grid = new_grid;
  m_cell_size = m_grid.getCellSize();

  generateVertices();
  generateEdges();

  return;
}

string EsriBathyGrid::getVarianceGridSpec() {

  // BUILD CONFIG SPEC

  unsigned int cix = m_grid.getCellVarIX("var");

  string tmpspc = m_grid.getConfigStr();

  int border_end = tmpspc.find("}") + 1;
  string spec = tmpspc.substr(0, border_end);

  spec += ",cell_size=" + to_string((int)m_cell_size) + ",";
  spec += "cell_vars=var:1000,";
  spec += "cell_min=var:0.00001,cell_max=var:" + to_string(getMaxVar());

  cout << tmpspc << endl;
  cout << spec << endl;

  // BUILD ELEMENTS SPEC

  // iterate through grid elements
  unsigned int cell_count = m_grid.size();
  for (int ix = 0; ix < cell_count; ix++) {

    string cell_spec;

    // get just the variance out
    double dval = m_grid.getVal(ix, cix);
    cell_spec += "var:" + doubleToStringX(dval);

    if (cell_spec != "") {
      cell_spec = ",cell=" + uintToString(ix) + ":" + cell_spec;
      spec += cell_spec;
    }
  }

  spec += ",label=cons_var";

  return (spec);
}

bool EsriBathyGrid::getAheadIDs(double curr_x, double curr_y,
                                int curr_direction, int (&ahead_i)[3],
                                int (&directions)[3]) {

  // int heading_vals[] = { 0, 45, 90, 135, 180, 225, 270, 315 };

  double x_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};
  double y_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};

  int xy_i[] = {0, 0, 0};

  x_vals[7] = curr_x - m_cell_size; //  xoo
  y_vals[7] = curr_y + m_cell_size; //  o o

  x_vals[0] = curr_x;               //  oxo
  y_vals[0] = curr_y + m_cell_size; //  o o

  x_vals[1] = curr_x + m_cell_size; //  oox
  y_vals[1] = curr_y + m_cell_size; //  o o

  x_vals[2] = curr_x + m_cell_size; //  ooo
  y_vals[2] = curr_y;               //  o x

  x_vals[3] = curr_x + m_cell_size; //  o o
  y_vals[3] = curr_y - m_cell_size; //  oox

  x_vals[4] = curr_x;               //  o o
  y_vals[4] = curr_y - m_cell_size; //  oxo

  x_vals[5] = curr_x - m_cell_size; //  o o
  y_vals[5] = curr_y - m_cell_size; //  xoo

  x_vals[6] = curr_x - m_cell_size; //  x o
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

bool EsriBathyGrid::getBehindIDs(double curr_x, double curr_y,
                                 int curr_direction, int (&behind_i)[5]) {

  double x_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};
  double y_vals[] = {0, 0, 0, 0, 0, 0, 0, 0};

  int xy_i[] = {0, 0, 0, 0, 0};

  x_vals[7] = curr_x - m_cell_size; //  xoo
  y_vals[7] = curr_y + m_cell_size; //  o o

  x_vals[0] = curr_x;               //  oxo
  y_vals[0] = curr_y + m_cell_size; //  o o

  x_vals[1] = curr_x + m_cell_size; //  oox
  y_vals[1] = curr_y + m_cell_size; //  o o

  x_vals[2] = curr_x + m_cell_size; //  ooo
  y_vals[2] = curr_y;               //  o x

  x_vals[3] = curr_x + m_cell_size; //  o o
  y_vals[3] = curr_y - m_cell_size; //  oox

  x_vals[4] = curr_x;               //  o o
  y_vals[4] = curr_y - m_cell_size; //  oxo

  x_vals[5] = curr_x - m_cell_size; //  o o
  y_vals[5] = curr_y - m_cell_size; //  xoo

  x_vals[6] = curr_x - m_cell_size; //  x o
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

int EsriBathyGrid::headingToDirection(double curr_heading) {

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

//---------------------------------------------------------
// Procedure: getGridAverageDepth()
double EsriBathyGrid::getGridAverageDepth() {
  vector<double> depth_vector = this->getDepthVector();
  double sum = 0.0;
  vector<double>::iterator it;
  for (it = depth_vector.begin(); it != depth_vector.end(); ++it) {
    sum += *it;
  }
  return (sum / static_cast<double>(depth_vector.size()));
}

bool EsriBathyGrid::getBounds(double &min_x, double &min_y, double &max_x,
                              double &max_y) {

  XYSquare bounding_square = m_grid.getSBound();
  min_x = bounding_square.get_min_x();
  min_y = bounding_square.get_min_y();

  max_x = bounding_square.get_max_x();
  max_y = bounding_square.get_max_y();

  return true;
}

//--------------------------------------------------------
// Procedure getDeltaSpec()
//           returns the deltas of each grid using the
//           XYGridUpdate format
//           Input: tolerance expressed as a fraction [0-1]
//                  that is the maximum allowable difference
//                  that is ignored.  I.e. if tolerance = 0.1
//                  then the delta is not reported if the value
//                  in any cell is less than 10% different than
//                  the value in the old grid.
//           Output (by reference) found_delta = true if found
//                  change and need to send, false if no change
//                  or error.
//           Often this procedure is followed by setOldGridToNew();

std::string EsriBathyGrid::getDeltaSpec(double tolerance, bool &found_delta) {

  unsigned int cell_count = m_grid.size();
  if (cell_count != m_old_grid.size()) {
    found_delta = false;
    return ("");
  }

  unsigned int cix_depth;
  unsigned int cix_variance;
  if (m_grid.hasCellVar("depth") and m_grid.hasCellVar("var")) {
    cix_depth = m_grid.getCellVarIX("depth");
    cix_variance = m_grid.getCellVarIX("var");
  } else {
    found_delta = false;
    return ("");
  }

  XYGridUpdate update;
  update.setGridName(m_grid.get_label());

  for (unsigned int idx = 0; idx < cell_count; idx++) {
    double new_depth = m_grid.getVal(idx, cix_depth);
    double old_depth = m_old_grid.getVal(idx, cix_depth);
    double delta_depth = new_depth - old_depth;
    if (abs(delta_depth) > (old_depth * tolerance)) {
      update.addUpdate(idx, "depth", delta_depth);
    }
    double new_var = m_grid.getVal(idx, cix_variance);
    double old_var = m_old_grid.getVal(idx, cix_variance);
    double delta_var = new_var - old_var;
    if (abs(delta_var) > (old_var * tolerance)) {
      update.addUpdate(idx, "var", delta_var);
    }
  }

  // Check that we found any changes (deltas).  No need to send anything
  // if not
  std::string str;
  if (update.valid()) {
    found_delta = true;
    str = update.get_spec();
  } else {
    found_delta = false;
    str = "";
  }

  return (str);
}

//-------------------------------------------------------
// Procedure setOldGridToNew()
void EsriBathyGrid::setOldGridToNew() {
  if (m_grid.size() > 0)
    m_old_grid = m_grid;

  return;
}

//-------------------------------------------------------
// Procedure getBoarderSegList();
//
XYSegList EsriBathyGrid::getBorderSegList() {

  string grid_spec = m_grid.get_spec();
  string grid_spec_to_process = grid_spec;
  string poly_spec_str;
  XYSegList seg_list;
  XYSegList null_seglist;

  // pts={-75,-50:-75,-220: ...  }, .....
  if (biteStringX(grid_spec_to_process, '=') == "pts") {
    // The next segment of the string is for the grid
    poly_spec_str = biteStringX(grid_spec, '}');
    string remainder = biteStringX(poly_spec_str, '{');

    vector<string> svector = parseString(poly_spec_str, ':');
    for (unsigned int i = 0; i < svector.size(); i++) {
      string x_str = biteStringX(svector[i], ',');
      double x_dbl = stod(x_str.c_str());
      double y_dbl = stod(svector[i].c_str());
      seg_list.add_vertex(x_dbl, y_dbl);
    }
  } else {
    return (null_seglist);
  }

  return (seg_list);
}

//-------------------------------------------------------
// Procedure getCellsTop()
//        returns all the ids of all the cells along the top
std::set<unsigned int> EsriBathyGrid::getCellsTop() {
  XYSegList border_seglist = getBorderSegList();
  if (border_seglist.size() < 4) {
    std::set<unsigned int> empty_seglist;
    return (empty_seglist);
  }

  double offset = 0.0; // small offset might be needed
                       // if line is horizontal
  double epsilon = 0.001;
  if ((border_seglist.get_vy(0) - border_seglist.get_vy(3)) < 0.001) {
    offset = -m_cell_size * .1;
  }

  double x1 = border_seglist.get_vx(0);
  double y1 = border_seglist.get_vy(0) + offset;
  double x2 = border_seglist.get_vx(3);
  double y2 = border_seglist.get_vy(3) + offset;

  return (getCellsBetween(x1, y1, x2, y2));
}

//-------------------------------------------------------
// Procedure getCellsBottom
//        returns all the ids of all the cells along the bottom
std::set<unsigned int> EsriBathyGrid::getCellsBottom() {
  XYSegList border_seglist = getBorderSegList();
  if (border_seglist.size() < 4) {
    std::set<unsigned int> empty_seglist;
    return (empty_seglist);
  }

  double offset = 0.0; // small offset might be needed
                       // if line is horizontal
  double epsilon = 0.001;
  if ((border_seglist.get_vy(1) - border_seglist.get_vy(2)) < 0.001) {
    offset = m_cell_size * .1;
  }

  double x1 = border_seglist.get_vx(1);
  double y1 = border_seglist.get_vy(1) + offset;
  double x2 = border_seglist.get_vx(2);
  double y2 = border_seglist.get_vy(2) + offset;

  return (getCellsBetween(x1, y1, x2, y2));
}

//----------------------------------------------------------
// Procedure: getCellsBetween(double x1, double y1, double x2, double y2)
std::set<unsigned int> EsriBathyGrid::getCellsBetween(double x1, double y1,
                                                      double x2, double y2) {
  std::set<unsigned int> set_to_return;
  // March along the line from vertex 1 to 2 and
  // add all the cells we find

  double dx = x2 - x1;
  double dy = y2 - y1;

  double x = x1;
  double y = y1;
  double len = sqrt(dx * dx + dy * dy);

  // incremental lengths to move along the line
  double lx = ((m_cell_size / 10.0) / len) * (x2 - x1);
  double ly = ((m_cell_size / 10.0) / len) * (y2 - y1);

  while (sqrt(pow((x1 - x), 2) + pow((y1 - y), 2)) < len) {
    // Check here
    int cell_id;
    bool cell_found = getCellID(x, y, cell_id);
    if (cell_found) {
      set_to_return.insert(cell_id);
    }

    // move by cell_size/10 along the line
    x = x + lx;
    y = y + ly;
  }

  return (set_to_return);
}

//--------------------------------------------------------
// Procedure:  getPathLength(path)
double EsriBathyGrid::getPathLength(std::vector<size_t> const &path) const {
  double length = 0.0;

  double x1, y1, x2, y2;
  for (unsigned int i = 0; i < (path.size() - 1); i++) {

    unsigned int index_i = static_cast<unsigned int>(path[i]);
    unsigned int index_i_plus_one = static_cast<unsigned int>(path[i + 1]);

    getCellXY(index_i, x1, y1);
    getCellXY(index_i_plus_one, x2, y2);

    double dx = x2 - x1;
    double dy = y2 - y1;
    length += sqrt(dx * dx + dy * dy);
  }

  return (length);
}

//---------------------------------------------------------
// Procedure: getGridLabel(string grid_spec)
//            extract the label from a grid spec

static string getGridLabel(string grid_spec) {
  vector<string> svector = parseString(grid_spec, ',');
  unsigned int length = svector.size();
  if (length < 1) {
    return ("");
  }
  // Get the last entry which should be the label
  if (biteStringX(svector[length - 1], '=') == "label") {
    return (svector[length - 1]);
  } else {
    return ("");
  }
}
