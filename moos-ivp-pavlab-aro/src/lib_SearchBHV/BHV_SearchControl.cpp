/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_SearchControl.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_SearchControl.h"

#include "ZAIC_PEAK.h"
#include "GeomUtils.h"
#include "AngleUtils.h"
#include <cmath>
#include "XYFormatUtilsPoly.h"
#include "OF_Coupler.h"


using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_SearchControl::BHV_SearchControl(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");
  
  // Initialize state variables
  m_osx = 0;
  m_osy = 0;
  m_ownship_in_region = false;
  m_region_center_x=0;
  m_region_center_y=0;
  m_cruise_speed = 0;
  m_voronoi=true;
  m_stoch=false;
  m_rand_heading=0;
  m_rand_speed=false;
  m_no_heading=true;
  m_outbound=true;
  m_mode="rotate";
  m_max_speed=3;
  m_ran=0;
  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING");
  addInfoVars("PROXONOI_POLY");
  addInfoVars("PROXONOI_REGION");
  addInfoVars("PROX_UP_REGION");
  addInfoVars("COVER");
}

//---------------------------------------------------------------
// Procedure: getMinRadius()
//   Purpose: Called to obtain the min radius of op region
//            returns a double of min distance

double BHV_SearchControl::getMinRadius()
{
  double min_rad=100000;
  //initialize the line paramters A, B, C in
  //Ax+By+C=0
  double A=0; 
  double B=0; 
  double C=0;
  double m = m_op_region.get_center_x();
  double n = m_op_region.get_center_y();

  for(unsigned int i=0; i<m_op_region.size(); i++) {
    double x1= m_op_region.get_vx(i);
    double y1= m_op_region.get_vy(i);
    double x2=0;
    double y2=0;
    if(i==m_op_region.size()-1){
      x2= m_op_region.get_vx(0);
      y2= m_op_region.get_vy(0);
    }
    else{
      x2= m_op_region.get_vx(i+1);
      y2= m_op_region.get_vy(i+1);
    }
    A=y1-y2;
    B=x2-x1;
    C=(x1*y2)-(x2*y1);
    double dist=abs((A*m)+(B*n)+C)/(sqrt((A*A)+(B*B)));
    if(dist < min_rad)
      min_rad=dist;
  }
  return min_rad;
}
//--------------------------------------------------------
//Update Ownship Position
bool BHV_SearchControl::updateOwnshipPosition()
{
//========================================================= 
  // Part 1: Update ownship position and check for errors
  //========================================================= 
  bool ok_x = true;
  bool ok_y = true;
  double new_osx = getBufferDoubleVal("NAV_X", ok_x);
  double new_osy = getBufferDoubleVal("NAV_Y", ok_y);

  if(!ok_y || !ok_y) {
    postEMessage("ownship NAV_X/Y info not found.");
    return(false);
  }

  //========================================================= 
  // Part 3: Update ownship position and the odometer
  //========================================================= 
  m_osx = new_osx;
  m_osy = new_osy;
  m_ownship_in_region = m_op_region.contains(m_osx, m_osy);
  if(m_ownship_in_region){
    m_in=1;
  }
  else{
    m_in=1;
  }

  return(true);


}

//---------------------------------------------------------------
// Procedure: Handle Op Region
bool BHV_SearchControl::handleConfigOpRegion(string polystr)
{
  XYPolygon new_poly = string2Poly(polystr);
  if(!new_poly.is_convex()){
    return(false);
  }
  m_op_region = new_poly;
  m_region_center_x=m_op_region.get_center_x();
  m_region_center_y=m_op_region.get_center_y();
  m_region_radius=m_op_region.max_radius();
  postMessage("THECENTX",m_region_center_x);
  postMessage("THECENTY",m_region_center_y);
    double max_dist_so_far = 0;
  double cx = m_op_region.get_center_x();
  double cy = m_op_region.get_center_y();

  for(unsigned int i=0; i<m_op_region.size(); i++) {
    double delta_x = cx - m_op_region.get_vx(i);
    double delta_y = cy - m_op_region.get_vy(i);
    double dist = hypot(delta_x, delta_y);
    if(dist > max_dist_so_far)
      max_dist_so_far = dist*cos((M_PI*45)/180);
      m_max_x=m_op_region.get_vx(i);
      m_max_y=m_op_region.get_vy(i);
  }
  m_region_radius=max_dist_so_far;
  postMessage("THERAD",m_region_radius);
  return(true);
}
//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_SearchControl::setParam(string param, string param_val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);
  
  bool handled = false;
  if(param == "speed"){
    handled = setNonNegDoubleOnString(m_cruise_speed, param_val);
  }
  else if(param == "op_region"){
    handled = handleConfigOpRegion(param_val);
  }
  else if(param == "spin_rad"){
    handled = setNonNegDoubleOnString(m_spin_rad, param_val);
  }
  else if(param == "speed_max"){
    handled = setNonNegDoubleOnString(m_max_speed, param_val);
  }
  else if(param == "stochastic"){
    handled = setBooleanOnString(m_stoch, param_val);
  }
  else if(param == "random_number"){
    handled = setIntOnString(m_ran, param_val);
  }
  else if(param == "mode"){
     m_mode=tolower(param_val);
     handled=true;
     if(m_mode=="stochastic"||m_mode=="rotate_speed"||m_mode=="stochastic"||m_mode=="stochastic_free_speed")
     m_rand_speed=true;
  }

  // If not handled above, then just return false;
  return(handled);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_SearchControl::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_SearchControl::onHelmStart()
{
  int seed = time(NULL)*m_ran;
  postMessage("SEED",double(seed));
  srand (seed);
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_SearchControl::onIdleState()
{
      string polystr = getBufferStringVal("PROX_UP_REGION");
      postMessage("TMP_DEBUG", polystr);
      string cov = getBufferStringVal("COVER");
      if(cov=="false"){
        m_voronoi=false;
      }
      if(m_voronoi){
      bool got =handleConfigOpRegion(polystr);
      }
      
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_SearchControl::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_SearchControl::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_SearchControl::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_SearchControl::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_SearchControl::onRunState()
{
  cout << "onRunState()" << endl;

    // Check for ok syntax in Proxonoi Poly. If convex, all is good.
    // If nonconvex poly with non-zero number of vertices, this is a
    // problem. Truly null polys (zero vertices) are fine, and mean
    // there just is not proxonoi poly to be used.
  // Part 1: Update ownship and proxonoi information
  bool ok = updateOwnshipPosition();
  if(!ok) {
    postMessage("VECTOR_DEBUG", "Unable to update ownship position");
    return(0);
  }

  // Part 1: Build the IvP function
    IvPFunction *ipf = buildOF();


  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.

  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}

IvPFunction *BHV_SearchControl::buildOF() 
{
  IvPFunction *ipf = 0;

  IvPFunction *spd_ipf = 0;  

////////////////////////////////////////////////////////
/////////////Rotation Mode//////////////////////////////
///////////////////////////////////////////////////////


  if(m_mode=="rotate"||m_mode=="rotate_speed"){
      //===================================================
      // Part 1: Build the Speed ZAIC
      //===================================================
        ZAIC_PEAK spd_zaic(m_domain, "speed");
          //determine the distance to the center of the region
          //this is not currently used but can be to create a speed gradient
          /*
          double mag1=((m_osy-m_region_center_y));
          double mag2=((m_osx-m_region_center_x));
          double mag=sqrt((mag1*mag1)+(mag2*mag2));
          double mag1n=((m_max_y-m_region_center_y));
          double mag2n=((m_max_x-m_region_center_x));
          double max_region=sqrt((mag1n*mag1n)+(mag2n*mag2n));
          */
        //Check to see if a random speed is requested
        //For rotate it will just rotate at the same random
        //Speed for the duration of the mission
        if(m_rand_speed){
          int sp=int(m_max_speed);
          m_cruise_speed=( rand() % sp + 1);
          m_rand_speed=false;
        }

        //Assign the ordered speed to the zaic
          double speedord=m_cruise_speed;
          double peak_width = speedord / 2;
          spd_zaic.setParams(m_cruise_speed, peak_width, 1.6, 20, 0, 100);
          spd_ipf = spd_zaic.extractIvPFunction();
          if(!spd_ipf)
            postWMessage("Failure on the SPD ZAIC via ZAIC_PEAK utility");
        


        //===================================================
        // Part 2: Build the Course ZAIC
        //===================================================
      
        ZAIC_PEAK crs_zaic(m_domain, "course");
          //First solve for the heading tangent to the center of the op_region
          double y=m_osy-m_region_center_y;
          double x=-1*(m_osx-m_region_center_x);
          double resultt = atan2 (y,x) * 180 / M_PI;
          if(resultt<0)
          resultt = (resultt + 360);
          //get distance to the center of op_region
          double dis=sqrt(x*x+y*y);

          //Here finding the minimum distance to op_region poly vertex
          double min_rad=getMinRadius();

          //Assume that the closest vertex lies at the corner
          //Take 80 percent of the min radius as the spin radius
          min_rad=.8*min_rad;
          postMessage("DISTC",dis-min_rad);
          postMessage("REGION_RAD",min_rad);
          //Check if os is outside this spin radius if so 
          //drive a tangent heading to the spin circle
          //if drifting correct the desired heading to maintain the circle
          if (dis>min_rad){
            double expanding=(dis-min_rad)/min_rad;
            expanding = 200*expanding;
            crs_zaic.setSummit(resultt+expanding);
          }
          else{
            crs_zaic.setSummit(resultt-90);
          }
          crs_zaic.setBaseWidth(90);
          crs_zaic.setValueWrap(true);
        IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);  
        if(!crs_ipf) 
          postWMessage("Failure on the CRS ZAIC");
        
        OF_Coupler coupler;
        ipf = coupler.couple(crs_ipf, spd_ipf, 0.5, 0.5);
        if(!ipf)
          postWMessage("Failure on the CRS_SPD COUPLER");
  }



////////////////////////////////////////////////////////  
/////////Stochastic Mode
/////////////////////////////////////////////////////////


  else if (m_mode=="stochastic"||m_mode=="stochastic_heading"){
    //===================================================
    // Part 1: Build the Speed ZAIC
    //===================================================
    ZAIC_PEAK spd_zaic(m_domain, "speed");
    //set to zaic speed to the current desired speed
    double speedord=m_cruise_speed;
    double peak_width = speedord *.25;
    spd_zaic.setParams(speedord, peak_width, 1.6, 20, 0, 100);
    spd_ipf = spd_zaic.extractIvPFunction();
    if(!spd_ipf)
      postWMessage("Failure on the SPD ZAIC via ZAIC_PEAK utility");
  
    //===================================================
    // Part 2: Build the Course ZAIC
    //===================================================
    //Calculate distance to center
    double mag1=((m_osy-m_region_center_y));
    double mag2=((m_osx-m_region_center_x));
    double mag=sqrt((mag1*mag1)+(mag2*mag2));
    ZAIC_PEAK crs_zaic(m_domain, "course");
    //get to heading to the center of the region
    double y=m_osy-m_region_center_y;
    double x=-1*(m_osx-m_region_center_x);
    double resultt = relAng(m_osx, m_osy,m_region_center_x, m_region_center_y);
    if(resultt<0)
    resultt = (resultt + 360);
    //Project Ownship position 5 steps in future to see if
    //Ownship will be out of the op region, if so the turn around
    //and drive towards the center of region
    double min_rad=getMinRadius();
    bool in_region=true;
    if(m_outbound){
      double dx=5*speedord*sin(m_rand_heading*M_PI/180);
      double dy=5*speedord*cos(m_rand_heading*M_PI/180);
      double p_osx=m_osx+dx;
      double p_osy=m_osy+dy;
      in_region = m_op_region.contains(p_osx, p_osy);
    }
    if(!in_region&&m_outbound){
      m_outbound=false;
      m_no_heading=true;
    }
    if(!m_outbound){
      int sp=int(m_max_speed);
      if(m_rand_speed){
        m_cruise_speed=( rand() % sp + 1);
      }
      crs_zaic.setSummit(resultt);
    }
    postMessage("CENTX", min_rad);
    double turn_rad=min_rad*.2;
    if (turn_rad<4){
      turn_rad=4;
    }
    if(!m_outbound&&mag<(turn_rad)){
      m_outbound=true;
    }
    if(m_outbound&&m_no_heading){
      m_rand_heading=( rand() % 360 + 1);
      if(m_rand_speed){
        int sp=int(m_max_speed);
        m_cruise_speed=( rand() % sp + 1);
      }
      m_no_heading=false;
    }
    if(m_outbound&&!m_no_heading){
      crs_zaic.setSummit(m_rand_heading);
    }

      crs_zaic.setPeakWidth(10);
      crs_zaic.setBaseWidth(180.0);
      crs_zaic.setSummitDelta(0);  
      crs_zaic.setValueWrap(true);
      IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);  
  
  
    if(!crs_ipf) 
      postWMessage("Failure on the CRS ZAIC");
    
    OF_Coupler coupler;
    ipf = coupler.couple(crs_ipf, spd_ipf, 0.5, 0.5);
    if(!ipf)
      postWMessage("Failure on the CRS_SPD COUPLER");


  }


  ///////////////////////////////////////////////////////
  /////////Full Stochastic Mode/////////////////////////
  //////////////////////////////////////////////////////



  else if (m_mode=="stochastic_free"||m_mode=="stochastic_free_speed"){
    //===================================================
    // Part 1: Build the Speed ZAIC
    //===================================================
    ZAIC_PEAK spd_zaic(m_domain, "speed");

    double speedord=m_cruise_speed;
    double peak_width = speedord *.25;
    spd_zaic.setParams(speedord, peak_width, 1.6, 20, 0, 100);
    spd_ipf = spd_zaic.extractIvPFunction();
    if(!spd_ipf)
      postWMessage("Failure on the SPD ZAIC via ZAIC_PEAK utility");
  
    //===================================================
    // Part 2: Build the Course ZAIC
    //===================================================
 
    ZAIC_PEAK crs_zaic(m_domain, "course");

    double y=m_osy-m_region_center_y;
    double x=-1*(m_osx-m_region_center_x);
    double resultt = relAng(m_osx, m_osy,m_region_center_x, m_region_center_y);


    //resultt=resultt+90;
    if(resultt<0)
    resultt = (resultt + 360);
    //Project Ownship position 2 steps in future to see if
    //Ownship will be out of the op region, if so the turn around
    //and drive toward the center of the region
        double dx1=2*speedord*sin(m_rand_heading*M_PI/180);
        double dy1=2*speedord*cos(m_rand_heading*M_PI/180);
        double p_osx1=m_osx+dx1;
        double p_osy1=m_osy+dy1;
    if(m_op_region.contains(p_osx1, p_osy1)){
      //Project Ownship position 10 steps in future to see if
      //Ownship will be out of the op region, if so calculate
      //a new random heading
      bool in_region=true;
      if(m_outbound){
        double dx=10*speedord*sin(m_rand_heading*M_PI/180);
        double dy=10*speedord*cos(m_rand_heading*M_PI/180);
        double p_osx=m_osx+dx;
        double p_osy=m_osy+dy;
        in_region = m_op_region.contains(p_osx, p_osy);
      }
      if(!in_region&&m_outbound){
        m_outbound=false;
        m_no_heading=true;
      }
      if(!m_outbound){
        if(m_rand_speed){
          int sp=int(m_max_speed);
          m_cruise_speed=( rand() % sp + 1);
        }
        m_outbound=true;
      }
      if(m_outbound&&m_no_heading){
        m_rand_heading=( rand() % 360 + 1);
        if(m_rand_speed){
          int sp=int(m_max_speed);
          m_cruise_speed=( rand() % sp + 1);
        }
        m_no_heading=false;
      }
      if(m_outbound&&!m_no_heading){
        crs_zaic.setSummit(m_rand_heading);
      }
    }
    else{
      crs_zaic.setSummit(resultt);
    }
      crs_zaic.setPeakWidth(10);
      crs_zaic.setBaseWidth(180.0);
      crs_zaic.setSummitDelta(0);  
      crs_zaic.setValueWrap(true);
      IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);  
  
  
  if(!crs_ipf) 
    postWMessage("Failure on the CRS ZAIC");
  
  OF_Coupler coupler;
  ipf = coupler.couple(crs_ipf, spd_ipf, 0.5, 0.5);
  if(!ipf)
    postWMessage("Failure on the CRS_SPD COUPLER");


  }
  
  return(ipf);
}
