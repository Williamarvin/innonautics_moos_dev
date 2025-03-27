/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PingSim.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <fstream>
#include "MBUtils.h"
#include "ACTable.h"
#include "PingSim.h"

#include "XYFormatUtilsConvexGrid.h"


using namespace std;

//---------------------------------------------------------
// Constructor

PingSim::PingSim()
{
}

//---------------------------------------------------------
// Destructor

PingSim::~PingSim()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool PingSim::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    double dval  = msg.GetDouble();
#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "NAV_X")
       m_nav_x = dval;
     else if(key == "NAV_Y")
       m_nav_y = dval;
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool PingSim::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool PingSim::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if (m_mode == "random") { // random values
  
    m_rand_dist = rand() % 20;
    m_rand_dist = (rand() % 2) + 18;
  
    Notify("PING_DISTANCE", m_rand_dist);
    Notify("PING_CONFIDENCE", 100);

  } else if (m_mode == "function") {  // Specify function per x and y
    
    double max_depth = 20.0;
    double val1 = sin( (m_nav_x + 75) * (4.0 * PI) / ( 185 - (-75)) );  // two waves in the grid
    double val2 = sin( (m_nav_y + 55) * (2.0 * PI) / (-220 - (-55)) );  // one wave in the grid
    m_calc_depth = (max_depth / 2.0) * val1 * val2 + (max_depth / 2.0 );
  
    Notify("PING_DISTANCE", m_calc_depth);
    Notify("PING_CONFIDENCE", 100);
    
  } else if (m_mode == "file") {   

    double var;
    double x,y;
    if (m_mirror_grid and m_mirror_set_up) {
      // mirror the nav_x and nav_y about the mirror line
      mirrorPoint(m_nav_x, m_nav_y, x, y);

    } else {
      x = m_nav_x;
      y = m_nav_y;
    }
    bool getdata = m_grid.getCellDataXY(x, y, m_file_depth, var);
    
    //  Add some white noise to the file
    unsigned long int tseed = time(NULL);
    unsigned long int pseed = getpid() + 1;
    
    unsigned int rseed = (tseed*pseed) % 50000;
    std::default_random_engine generator (rseed);
    
    std::normal_distribution<double> distribution (0.0,1.0);
    m_noise_added = distribution(generator);
    m_file_depth = m_file_depth + m_noise_added;

    if (getdata) {
      cout << m_file_depth << "got data" << endl;
      Notify("PING_DISTANCE", m_file_depth);
      Notify("PING_CONFIDENCE", 100);
    } else {
      cout << "no data" << endl;
      Notify("PING_DISTANCE", -1);
      Notify("PING_CONFIDENCE", 0.0);
    }

  }
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool PingSim::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  string grid_config;

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "mode") {
      handled = true;
      if (m_mode == "file" || m_mode == "random" || m_mode == "function")
	m_mode = value;
      else
	m_mode = "function";
      
    } else if(param == "filename") {
      handled = true;
      m_filename = value;

    } else if(param == "mirror_grid") {
      handled = isBoolean(value);
      m_mirror_grid = (value == "true");
      
    } else if (param == "grid_config") {
      handled = true;
      unsigned int len = grid_config.length();
      if((len > 0) && (grid_config.at(len-1) != ','))
	grid_config += ",";
      grid_config += value;
    }
    
    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  double LatOrigin = 0.0;
  double LonOrigin = 0.0;

  // Get Latitude Origin from .MOOS Mission File
  bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
  if(!latOK) {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return(false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
  if(!lonOK){
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return(false);
  }
  

  if (m_mode == "file") {

    if (m_filename == "")
      m_mode = "random";
    else
      populateFromFile(LatOrigin,LonOrigin);
   
  }

  // assemble the mirror line which is half way
  // between the left and right.
  if (grid_config != "") {
      XYConvexGrid temp_grid = string2ConvexGrid(grid_config);

      if(temp_grid.size() == 0) {
	reportConfigWarning("Unsuccessful ConvexGrid construction.");
      } else {
	
	m_grid_op_region = EsriBathyGrid(temp_grid,LatOrigin,LonOrigin,0);
	XYSegList border_sl = m_grid_op_region.getBorderSegList();
	
	m_x1 = 0.5 * ( border_sl.get_vx(0) + border_sl.get_vx(3) );
	m_y1 = 0.5 * ( border_sl.get_vy(0) + border_sl.get_vy(3) );
	m_x2 = 0.5 * ( border_sl.get_vx(1) + border_sl.get_vx(2) );
	m_y2 = 0.5 * ( border_sl.get_vy(1) + border_sl.get_vy(2) );
	m_mirror_set_up = true;
      }
       
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void PingSim::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool PingSim::buildReport() 
{
  m_msgs << "Random depth = " << m_rand_dist << endl;
  m_msgs << "Calculated depth = " << m_calc_depth << endl;
  m_msgs << "File depth  = " << m_file_depth << endl;
  m_msgs << "Noise added = " << m_noise_added << endl;
  
  return(true);
}

//------------------------------------------------------------
// Procedure: populateFromFile()
//
// puts grid from file into m_filegrid
//
bool PingSim::populateFromFile(int lat, int lon) 
{

  // open file
  ifstream newfile;

  newfile.open(m_filename,ios::in);
  
  if (newfile.is_open()) {

    // read in file to string
    
    string poly_spec;
    getline(newfile, poly_spec); // discard first line
    getline(newfile, poly_spec); // get second line    
    newfile.close();

    // parse string and convert

    // get cell size string
    int cell_size_end = poly_spec.find(" # ");
    string cell_size_str = poly_spec.substr(0, cell_size_end);
    poly_spec = poly_spec.substr(cell_size_end+3);

    //cout << poly_spec << endl;
  
    // get border string
    int border_start = poly_spec.find("{");
    int border_end = poly_spec.find(" # ");
    string border_str = poly_spec.substr(border_start, border_end-border_start);

    string convex_spec = "pts=" + border_str + "," + cell_size_str + ",";
    
    convex_spec += "cell_vars=depth:0:var:0,cell_min=depth:0,cell_max=depth:30,cell_min=var:0.0001,cell_max=var:100,";

    int cell_start = poly_spec.find("@");

    string cell_data = poly_spec.substr(cell_start+1);


    // reformat cell data
    size_t pos = 0;
    string token;
    int colon_pos;
    
    while ((pos = cell_data.find(',')) != std::string::npos) {
      
      token = cell_data.substr(0, pos);
      colon_pos = token.find(":");
      convex_spec += token.substr(0,colon_pos) + ":depth" + token.substr(colon_pos) + "var:100,";     
      cell_data.erase(0, pos+1);
      
    }

    colon_pos = token.find(":");
    convex_spec += cell_data.substr(0,colon_pos) + ":depth" + cell_data.substr(colon_pos) + "var:100";     
    XYConvexGrid temp_grid = string2ConvexGrid(convex_spec);

    if(temp_grid.size() == 0)
      reportConfigWarning("Unsuccessful ConvexGrid construction.");
     
    temp_grid.set_label("file");
        
    m_grid = EsriBathyGrid(temp_grid,lat,lon,0);

    //Notify("VIEW_GRID",m_grid.getGridSpec());

    return true;
  }

  cout << "file not found" << endl;

  m_mode = "function";


  return false;

}


//  Procedure:  mirrorPoint(p1x, p1y, &p2x, &p2y).
//              mirrors point p1 about the line
//              defined by m_x1, m_y1 to m_x2, m_y2
//              returns p2x and p2y by reference 
void PingSim::mirrorPoint(double p1x, double p1y, double &p2x, double &p2y) {
  double dx = m_x2 - m_x1;
  double dy = m_y2 - m_y1;

  double a = (dx*dx - dy*dy)  / (dx*dx + dy*dy);
  double b = 2*dx*dy / ( dx*dx + dy*dy );

  p2x = a*(p1x - m_x1) + b*(p1y - m_y1) + m_x1;
  p2y = b*(p1x - m_x1) - a*(p1y - m_y1) + m_y1;
  
  return;
}
