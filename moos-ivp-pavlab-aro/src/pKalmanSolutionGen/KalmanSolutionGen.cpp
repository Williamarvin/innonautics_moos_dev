/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: KalmanSolutionGen.cpp                                        */
/*    DATE: May 26, 2020                                */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "KalmanSolutionGen.h"

using namespace std;

//---------------------------------------------------------
// Constructor

KalmanSolutionGen::KalmanSolutionGen()
{
  
  m_x=0;
  m_xo=0;
  m_y=0;
  m_yo=0;
  m_vx=0;
  m_vy=0;
  m_time=-1000;
  //The Updated Values
  m_x1=0;
  m_y1=0;
  m_vx1=0;
  m_vy1=0;
  m_got_speed=false;
  m_time1=-1000;
  //m_P[2][2]={{10,0}, {0,1}};
  m_P[0][0]=100;
  m_P[0][1]=0;
  m_P[1][0]=0;
  m_P[1][1]=1000;
  m_Py[0][0]=100;
  m_Py[0][1]=0;
  m_Py[1][0]=0;
  m_Py[1][1]=1000;
  //Config Parameters Check
  m_defined=false;
   m_px00=0;
   m_px11=0;
   m_py00=0;
   m_py11=0;
   m_meas_noise=10;
   m_init_noise=10;
   m_got_report=false;
   m_got_solution=false;
   m_detections=0;
   m_solutions=0;
   m_heading =0;
   m_speed=0;
}

//---------------------------------------------------------
// Destructor

KalmanSolutionGen::~KalmanSolutionGen()
{
}
//---------------------------------------------------------
// Procedure: handleReport

void KalmanSolutionGen::handleReport(string report)
{
    string x = tolower(biteStringX(report, ';'));
    string y = tolower(biteStringX(report, ';'));
    string time= tolower(biteStringX(report, ';'));
    string heading=tolower(biteStringX(report, ';'));
    string speed=report;
    bool g_x;
    bool g_y;
    bool g_t1;
    if(!m_got_speed){
      if(m_time<0){
        g_x =setDoubleOnString(m_x, x);
        g_y =setDoubleOnString(m_y, y);
        g_t1 =setDoubleOnString(m_time, time);
        initModel(m_x,m_y,2.0,2.0,m_time);
      }
      /*else {
        double x1=0;
        double y1 =0;
        double t1 =0;
        g_x=setDoubleOnString(x1, x);
        g_y =setDoubleOnString(y1, y);
        g_t1 =setDoubleOnString(t1, time);
        if(t1>m_time){
          double vx=(x1-m_x)/(t1-m_time);
          double vy=(y1-m_y)/(t1-m_time);
          initModel(x1,y1,vx,vy,t1);
        }
      }*/

    }
    else {
      g_x =setDoubleOnString(m_x1, x);
      g_y =setDoubleOnString(m_y1, y);
      g_t1 =setDoubleOnString(m_time1, time);
      g_x =setDoubleOnString(m_heading, heading);
      g_y =setDoubleOnString(m_speed, speed);
      if(m_time1>m_time){
      executeKalmanX();
      executeKalmanY();
      postEstimates();
      m_time=m_time1;
      }
    }
}

//---------------------------------------------------------
// Procedure: initModel
void KalmanSolutionGen::initModel(double x, double y, double vx, double vy, double time)
{
  m_x=x;
  m_xo=x;
  m_y=y;
  m_yo=y;
  m_vx=vx;
  m_vy=vy;
  m_time=time;
  m_got_speed=true;


}
//---------------------------------------------------------
// Procedure: initModel
void KalmanSolutionGen::initializeSolution()
{
  m_x=0;
  m_y=0;
  m_vx=0;
  m_vy=0;
  m_time=-1000;
  //The Updated Values
  m_x1=0;
  m_y1=0;
  m_vx1=0;
  m_vy1=0;
  m_got_speed=false;
  m_time1=-1000;
  //Set p Matrix
  if(!m_defined){
    m_P[0][0]=100;
    m_P[0][1]=0;
    m_P[1][0]=0;
    m_P[1][1]=1000;
    m_Py[0][0]=100;
    m_Py[0][1]=0;
    m_Py[1][0]=0;
    m_Py[1][1]=1000;
  }
  else{
    m_P[0][0]=m_px00;
    m_P[0][1]=0;
    m_P[1][0]=0;
    m_P[1][1]= m_px11;
    m_Py[0][0]= m_py00;
    m_Py[0][1]=0;
    m_Py[1][0]=0;
    m_Py[1][1]=m_py11;
  }
}
//---------------------------------------------------------
// Procedure: executeKalmanX

void KalmanSolutionGen::executeKalmanX()
{

  double delta_tc=m_time1-m_time;
  double x_pred=0;
  double vx_pred=0;


  double p_pred[2][2]={{0,0}, {0,0}};
  double R[2][2]={{0,0}, {0,0}};
  double p_pred1[2][2]={{0,0}, {0,0}};


  double F[2][2]={{1,delta_tc}, {0,1}};
  double F_T[2][2]={{1,0}, {delta_tc,1}};

  double G[1][2]={0,delta_tc};

  double wk[2][2]={{m_init_noise,0}, {0,m_init_noise}};
  double vk[2][2]={{m_meas_noise,0}, {0,5*m_meas_noise/(delta_tc*delta_tc)}};
  //double vk=m_meas_noise;
  double accel=.001;
  //First Step is make predicted values
  //Starting with the position and velocitx
  x_pred=m_x+(m_vx*delta_tc);
  vx_pred=m_vx;
  //vx_pred=vx_pred+(delta_tc*accel);
  for(int i =0; i<2; i++){
    for(int j=0; j<2; j++){
      double tot=0;
        for (int inner = 0; inner < 2; inner++) {
          tot =tot+ (F[i][inner] * m_P[inner][j]);
        }
        p_pred1[i][j]=tot;
      
    }
  }
  for(int i =0; i<2; i++){
    for(int j=0; j<2; j++){
      double tot=0;
        for (int inner = 0; inner < 2; inner++) {
                 tot= tot+(p_pred1[i][inner] * F_T[inner][j]);
        }
      p_pred[i][j]=tot;
    }
  }
  for(int i =0; i<2; i++){
    for(int j=0; j<2; j++){
     
      p_pred[i][j] = p_pred[i][j] + wk[i][j];
      
    }
  }

  Notify("PRED_x",p_pred[0][0]);
  Notify("PRED_xd",p_pred[1][1]);
  //////////////////////////////////////////////////////////////
  /////Done with Predictions///////////////////////
  //Now solve for the k Matrix values k1 and k2
  /*
  double k1=p_pred[0][0]*(1/(p_pred[0][0]+(vk)));
  double k2=p_pred[1][0]*(1/(p_pred[0][0]+(vk)));
  */
 for(int i =0; i<2; i++){
  for(int j=0; j<2; j++){
      R[i][j] = p_pred[i][j] + vk[i][j];
    }
  }
  /*
inverse(A)= det(A)*|d  -b|
                   |-c  a|

  */
 //Solve the inverse of R
  double determ=((R[0][0]*R[1][1])-(R[0][1]*R[1][0]));
  determ=1/determ;

   double a=determ*R[1][1];
  double b=determ*-1*R[0][1];
  double c=determ*-1*R[1][0];
  double d=determ*R[0][0];

  R[0][0]=a;
  R[0][1]=b;
  R[1][0]=c;
  R[1][1]=d;
 //Solve for K
  double K[2][2]={{0,0}, {0,0}};
  for(int i =0; i<2; i++){
    for(int j=0; j<2; j++){
      double tot=0;
        for (int inner = 0; inner < 2; inner++) {
                tot = tot+(p_pred[i][inner] * R[inner][j]);
        }
        K[i][j]=tot;
    }
  }
  ///Now Solve for corrected position and speed
  double measured_x=m_x1;
  double measured_vx=abs((abs(measured_x)-abs(m_x)))/delta_tc;
  if(measured_x<m_x){
    measured_vx=-1*measured_vx;
  }
  double first=measured_x-x_pred;
  double second=measured_vx-vx_pred;
  double correction_x=(K[0][0]*first)+(K[0][1]*second);
  double correction_vx=(K[1][0]*first)+(K[1][1]*second);

Notify("A",correction_vx);

  double x_corr=x_pred+correction_x;
  double vx_corr=vx_pred+correction_vx;
  //Now solve for updated m_P
  //double K_M[2][2]={{1-k1,0}, {0-k2,1}};

  K[0][0]=1-K[0][0];
  K[1][1]=1-K[1][1];
  K[1][0]=0-K[1][0];
  K[0][1]=0-K[0][1];

  double _Pp[2][2]={{0,0}, {0,0}};
  _Pp[0][0]=0;
  _Pp[1][1]=0;
  _Pp[0][1]=0;
  _Pp[1][0]=0;

  for(int i =0; i<2; i++){
    for(int j=0; j<2; j++){
      double tot=0;
        for (int inner = 0; inner < 2; inner++) {
                tot = tot+(K[i][inner] * p_pred[inner][j]);
        }
        _Pp[i][j]=tot;
    }
  }
  m_P[0][0]=_Pp[0][0];
  m_P[1][1]=_Pp[1][1];
  m_P[0][1]=_Pp[0][1];
  m_P[1][0]=_Pp[1][0];
  //Now reset all original values to corrected
  m_x=x_corr;
  m_vx=vx_corr;

}

//---------------------------------------------------------
// Procedure: executeKalmanY

void KalmanSolutionGen::executeKalmanY()
{
    double delta_tc=m_time1-m_time;
  double y_pred=0;
  double vy_pred=0;


  double p_pred[2][2]={{0,0}, {0,0}};
  double R[2][2]={{0,0}, {0,0}};
  double p_pred1[2][2]={{0,0}, {0,0}};


  double F[2][2]={{1,delta_tc}, {0,1}};
  double F_T[2][2]={{1,0}, {delta_tc,1}};

  double G[1][2]={0,delta_tc};

  double wk[2][2]={{m_init_noise,0}, {0,m_init_noise}};
  double vk[2][2]={{m_meas_noise,0}, {0,5*m_meas_noise/(delta_tc*delta_tc)}};
  //double vk=m_meas_noise;
  double accel=.001;
  //First Step is make predicted values
  //Starting with the position and velocity
  y_pred=m_y+(m_vy*delta_tc);
  vy_pred=m_vy;
  //vy_pred=vy_pred+(delta_tc*accel);
  for(int i =0; i<2; i++){
    for(int j=0; j<2; j++){
      double tot=0;
        for (int inner = 0; inner < 2; inner++) {
          tot =tot+ (F[i][inner] * m_P[inner][j]);
        }
        p_pred1[i][j]=tot;
      
    }
  }
  for(int i =0; i<2; i++){
    for(int j=0; j<2; j++){
      double tot=0;
        for (int inner = 0; inner < 2; inner++) {
                 tot= tot+(p_pred1[i][inner] * F_T[inner][j]);
        }
      p_pred[i][j]=tot;
    }
  }
  for(int i =0; i<2; i++){
    for(int j=0; j<2; j++){
     
      p_pred[i][j] = p_pred[i][j] + wk[i][j];
      
    }
  }

  Notify("PRED_y",p_pred[0][0]);
  Notify("PRED_yd",p_pred[1][1]);
  //////////////////////////////////////////////////////////////
  /////Done with Predictions///////////////////////
  //Now solve for the k Matriy values k1 and k2
  /*
  double k1=p_pred[0][0]*(1/(p_pred[0][0]+(vk)));
  double k2=p_pred[1][0]*(1/(p_pred[0][0]+(vk)));
  */
 for(int i =0; i<2; i++){
  for(int j=0; j<2; j++){
      R[i][j] = p_pred[i][j] + vk[i][j];
    }
  }
  /*
inverse(A)= det(A)*|d  -b|
                   |-c  a|

  */
 //Solve the inverse of R
  double determ=((R[0][0]*R[1][1])-(R[0][1]*R[1][0]));
  determ=1/determ;

   double a=determ*R[1][1];
  double b=determ*-1*R[0][1];
  double c=determ*-1*R[1][0];
  double d=determ*R[0][0];

  R[0][0]=a;
  R[0][1]=b;
  R[1][0]=c;
  R[1][1]=d;
 //Solve for K
  double K[2][2]={{0,0}, {0,0}};
  for(int i =0; i<2; i++){
    for(int j=0; j<2; j++){
      double tot=0;
        for (int inner = 0; inner < 2; inner++) {
                tot = tot+(p_pred[i][inner] * R[inner][j]);
        }
        K[i][j]=tot;
    }
  }
  ///Now Solve for corrected position and speed
  double measured_y=m_y1;
  double measured_vy=abs((abs(measured_y)-abs(m_y)))/delta_tc;
  if(measured_y<m_y){
    measured_vy=-1*measured_vy;
  }
  double first=measured_y-y_pred;
  double second=measured_vy-vy_pred;
  double correction_y=(K[0][0]*first)+(K[0][1]*second);
  double correction_vy=(K[1][0]*first)+(K[1][1]*second);

Notify("A",correction_vy);

  double y_corr=y_pred+correction_y;
  double vy_corr=vy_pred+correction_vy;
  //Now solve for updated m_P
  //double K_M[2][2]={{1-k1,0}, {0-k2,1}};

  K[0][0]=1-K[0][0];
  K[1][1]=1-K[1][1];
  K[1][0]=0-K[1][0];
  K[0][1]=0-K[0][1];

  double _Pp[2][2]={{0,0}, {0,0}};
  _Pp[0][0]=0;
  _Pp[1][1]=0;
  _Pp[0][1]=0;
  _Pp[1][0]=0;

  for(int i =0; i<2; i++){
    for(int j=0; j<2; j++){
      double tot=0;
        for (int inner = 0; inner < 2; inner++) {
                tot = tot+(K[i][inner] * p_pred[inner][j]);
        }
        _Pp[i][j]=tot;
    }
  }
  m_Py[0][0]=_Pp[0][0];
  m_Py[1][1]=_Pp[1][1];
  m_Py[0][1]=_Pp[0][1];
  m_Py[1][0]=_Pp[1][0];
  //Now reset all original values to corrected
  m_y=y_corr;
  m_vy=vy_corr;
}
  //---------------------------------------------------------
// Procedure: postEstimates

void KalmanSolutionGen::postEstimates()
{
  double heading_est=(atan2 (m_vx,m_vy) * 180 / M_PI);
  if(heading_est<0){
    heading_est=heading_est+360;
  }
  double dx2=m_vx*m_vx;
  double dy2=m_vy*m_vy;
  double delta_d=sqrt(dx2+dy2);
  double speed_est=delta_d;
  double s_error=speed_est-m_speed;
  double h_error=heading_est-m_heading;
  Notify("HEADING_ESTIMATE_KAL", heading_est);
  Notify("SPEED_ESTIMATE_KAL", speed_est);
  Notify("HEADING_ESTIMATE", heading_est);
  Notify("SPEED_ESTIMATE", speed_est);
  Notify("HEADING_ERROR", h_error);
  Notify("SPEED_ERROR", s_error);
  m_got_solution =true;
}
//---------------------------------------------------------
// Procedure: OnNewMail

bool KalmanSolutionGen::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "DETECTION_REPORT"){
       string sval  = msg.GetString();
       m_got_report=true;
       handleReport(sval);
     } 
      else if(key == "FINISH_MISSION"){
       string sval  = msg.GetString();
        if(sval=="true"){
          if(m_got_report){
            m_detections=m_detections+1;
            Notify("SONAR_DETECTION",m_detections);
            m_got_report=false;
          }
          if(m_got_solution){
            m_solutions=m_solutions+1;
            Notify("NUM_SOLUTIONS",m_solutions);
            m_got_solution=false;
          }
          initializeSolution();
        }
     }


     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool KalmanSolutionGen::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool KalmanSolutionGen::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool KalmanSolutionGen::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

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
    /*
    m_defined=false;
   m_px00=0;
   m_px11=0;
   m_py00=0;
   m_py11=0;
   m_meas_noise=10;
   m_init_noise=1;


    */
    bool handled = false;
    if(param == "target_starting_x") 
      handled = setNonNegDoubleOnString(m_x, value);
    else if(param == "target_starting_y") 
      handled = setNonNegDoubleOnString(m_y, value);
    else if(param == "start_with_model")
      handled = setBooleanOnString(m_defined, value);
    else if(param == "px_00")
      handled = setNonNegDoubleOnString(m_px00, value);
    else if(param == "px_11")
      handled = setNonNegDoubleOnString(m_px11, value);
    else if(param == "py_00")
      handled = setNonNegDoubleOnString(m_py00, value);
    else if(param == "py_11")
      handled = setNonNegDoubleOnString(m_py11, value);
    else if(param == "wk")
      handled = setNonNegDoubleOnString(m_init_noise, value);
    else if(param == "vk")
      handled = setNonNegDoubleOnString(m_meas_noise, value);

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void KalmanSolutionGen::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
   Register("DETECTION_REPORT", 0);
   Register("FINISH_MISSION", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool KalmanSolutionGen::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: KalmanSolution                           " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "============================================" << endl;
  m_msgs << "Last X pos corrected = " << m_x<<endl;
  m_msgs << "============================================" << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Last x Velocity corrected = " << m_vx<<endl;
  m_msgs << "============================================" << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Last Y pos corrected = " << m_y<<endl;
  m_msgs << "============================================" << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Last y Velocity corrected = " << m_vy<<endl;
  m_msgs << "============================================" << endl;
  m_msgs << "============================================" << endl;

    // output each array element's value
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            m_msgs << "Element at P[" << i
                 << "][" << j << "]: ";
            m_msgs << m_P[i][j]<<endl;
        }
    }
  m_msgs << "============================================" << endl;
  m_msgs << "============================================" << endl;
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            m_msgs << "Element at Py[" << i
                 << "][" << j << "]: ";
            m_msgs << m_Py[i][j]<<endl;
        }
    }

m_msgs << "============================================" << endl;
  return(true);
}




