/************************************************************/
/*    NAME: Craig Evans                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SearchDetect.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SearchDetect.h"
#include "LogicUtils.h"
#include "NodeRecordUtils.h"
using namespace std;

//---------------------------------------------------------
// Constructor

SearchDetect::SearchDetect()
{
  m_hit=false;
  m_numhit=0;
  m_p=0;
  m_cpaunder=0;
  m_rand_val=0;
  m_time_elapse=0;
  m_time_dif=45;
  m_time1=0;
}

//---------------------------------------------------------
// Destructor

SearchDetect::~SearchDetect()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SearchDetect::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval  = msg.GetString(); 


    if(key == "NODE_REPORT") 
      handleMailNodeReport(sval);
    else if(key == "MISSION_START"){ 
      if(sval=="true"){
        m_mission_start=true;
      }
    }
    else if(key == "FINISH_MISSION"){ 
      if(sval=="true"){
        if(m_prob){
          if(m_detection_run[0]>=2){
            getEstimate();
          }
          m_first_report="";
          m_second_report="";
        }
        m_map_vname_detections.clear();
        for(int i=0; i<m_first_detection.size(); i++){
        m_first_detection[i]=true;
        m_detection_run[i]=0;
        m_current_vname[i]="";
        }

      }
    }
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SearchDetect::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SearchDetect::Iterate()
{
  AppCastingMOOSApp::Iterate();
  if(m_mission_start){
  m_cpa_monitor.examineAndReport();
  m_cpa_monitor.setIteration(m_iteration);
    unsigned int events = m_cpa_monitor.getEventCount();
    for(unsigned int ix=0; ix<events; ix++) {
      CPAEventNew event = m_cpa_monitor.getEvent(ix);
      handleCPAEventNew(event);
    }

  m_cpa_monitor.clear();
  }
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SearchDetect::OnStartUp()
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

    bool handled = false;
    if(param == "detect_range") 
      handled = initRange(value);
    else if(param == "target_name") 
      handled = setTarget(value);
    else if(param == "encounter_range") 
      handled = setNonNegDoubleOnString(m_encounter_dist, value);
    else if(param == "ignore_group") 
      handled = m_cpa_monitor.addIgnoreGroup(value);
    else if(param == "reject_group") 
      handled = m_cpa_monitor.addRejectGroup(value);
    else if(param == "sigma")
      handled = setNonNegDoubleOnString(m_sigma, value);
    else if(param == "probability_detect")
      handled = setBooleanOnString(m_prob, value);
    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
    
  m_cpa_monitor.setReportRange(m_encounter_dist);
  m_cpa_monitor.setIgnoreRange(m_encounter_dist * 1.5);
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void SearchDetect::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
  Register("NODE_REPORT", 0);
  Register("FINISH_MISSION");
  Register("MISSION_START");
}



//------------------------------------------------------------
// Procedure: handleMailNodeReport()

void SearchDetect::handleMailNodeReport(string sval)
{
  if(m_mission_start){
  // Part 1: inject the node report into the CPAMonitor
  bool ok = m_cpa_monitor.handleNodeReport(sval);
  if(!ok) 
    reportRunWarning("Unhandled Node Report:" + sval);

  // Part 2: The first time we deal with a particular vehicle, we 
  // post the parameter summary to be sent to that vehicle. We do
  // this now rather than at startup at this app, since we can't 
  // be sure that the other vehicle is online when this app starts
  // up. But now that we have received a node report from that 
  // vehicle, we can be pretty sure it will get this one-time msg.
  string vname = tokStringParse(sval, "NAME", ',', '=');
  if(!vectorContains(m_notified_vehicles, vname)) {
    m_notified_vehicles.push_back(vname);
    vname = toupper(vname);
  }
  }
}


void SearchDetect::handleCPAEventNew(CPAEventNew event)
{
  //===========================================================
  // Part 1: Get the cpa distance
  //===========================================================
  string v1   = tolower(event.getVName1());
  string v2   = tolower(event.getVName2());
  double cpa=100000000;
  if(v1==m_target_name||v2==m_target_name){
    cpa  = event.getCPA();
  }
  else{
    cpa =100000000;
  }
  double midx = event.getX();
  double midy = event.getY();
  string cpas = doubleToStringX(cpa,2);
  double alpha = event.getAlpha();
  double beta  = event.getBeta();
  
    //===========================================================
  // Part 2: Check if the cpa is less than detection distance 
  // and includes the target_name
  //===========================================================
 //=====================================================
 //Here evaluate whether using probablistic detection or
 //using discrete
 //======================================================
 if(!m_prob){
        for(int i=0; i<m_detection_dist.size();i++){
          if(cpa <= m_detection_dist[i]) {
            m_map_vname_detections[v1]++;
            m_map_vname_detections[v2]++;
                string note_="DETECTION";
                m_detection_run[i]++;
                if(m_first_detection[i]){
                  if (v1==m_target_name){
                  m_current_vname[i]=v2;
                  }
                  else{
                    m_current_vname[i]=v1;
                  }
                  Notify(note_,doubleToStringX(m_detection_dist[i]));
                  m_first_detection[i]=false;
                  Notify("FINAL", doubleToStringX(m_detection_dist[i])+"_M_DETECTION");
                  string mess = doubleToStringX(m_detection_dist[i])+"_M_DETECTION";
                  m_detection_total[i]++;
                  Notify(mess,to_string(m_detection_total[i]));
                }
                else if(m_detection_run[i]==2&&m_current_vname[i]!=v1&&m_current_vname[i]!=v2){
                  string mess = doubleToStringX(m_detection_dist[i])+"_M_DOUBLE_DETECTION";
                  m_more_than_one[i]=m_more_than_one[i]+1;
                  Notify(mess,m_more_than_one[i]);

                }
          }
        }
 }
  //=====================================================
 //If using probablistic detection
 //calculate probability based on distance
 //======================================================
 else{
   if(cpa <= m_detection_dist.back()) {
     m_cpaunder=cpa;
      double exponent=(cpa*cpa)/(2*(m_sigma*m_sigma));
      double p=exp(-1*exponent);
      m_p=p;
      m_hit=false;
      double result=(float) rand()/RAND_MAX;
      m_rand_val=result;
        if (result<p){
            m_hit=true;
        }
      
   }
   if(m_hit){
     
     string one_report;
     string add_report;
     m_numhit=m_numhit+1;
     string note_="DETECTION";
     m_hit=false;
     m_detection_run[0]++;
     double current_time=0;
/////////////////////Set time of event/////////////////////

      if (v1==m_target_name&&m_detection_run[0]>1){
        string rep=event.getN2();
          NodeRecord mess = string2NodeRecord(rep);
          current_time= mess.getTimeStamp();
      }
      else if(m_detection_run[0]>1){
          string rep=event.getN1();
          NodeRecord mess = string2NodeRecord(rep);
          current_time= mess.getTimeStamp();
      }

/////////////////////Set time of event/////////////////////

     m_time_elapse=current_time-m_time1;
      if(m_first_detection[0]){
      if (v1==m_target_name){
      m_current_vname[0]=v2;
        one_report=event.getN2();
        m_first_report=one_report;
          NodeRecord first = string2NodeRecord(m_first_report);
          m_time1= first.getTimeStamp();
      }
      else{
        m_current_vname[0]=v1;
        one_report=event.getN1();
        m_first_report=one_report;
          NodeRecord first = string2NodeRecord(m_first_report);
          m_time1= first.getTimeStamp();
      }
//This is the first detection so notify all variables
          Notify(note_,doubleToStringX(m_detection_dist[0]));
          m_first_detection[0]=false;
          Notify("FINAL", doubleToStringX(m_detection_dist[0])+"_M_DETECTION");
          string mess = doubleToStringX(m_detection_dist[0])+"_M_DETECTION";
          m_detection_total[0]++;
          Notify("NUM_DETECTIONS",to_string(m_detection_total[0]));


    }
    else if(m_detection_run[0]>1&&m_current_vname[0]!=v1&&m_current_vname[0]!=v2){
      string two_report;
          m_more_than_one[0]=m_more_than_one[0]+1;
          Notify("NUM_OF_DOUBLE_DETECTIONS",m_more_than_one[0]);
                if (v1==m_target_name){
                    two_report=event.getN1();
                    m_second_report=event.getN2();
                    m_target_report=event.getN1();
                }
                  else{
                    two_report=event.getN2();;
                    m_second_report=event.getN1();;
                    m_target_report=event.getN2();
                  }
          //getEstimate();
    }
   }
 }
}
//------------------------------------------------------------
// Procedure: initRange(string)
// Parses out the detection range increments

bool SearchDetect::initRange(string range)
{
string a=(biteStringX(range,':'));
string b=(biteStringX(range,':'));
string c=range;



double start=atof(a.c_str());
double end=atof(b.c_str());
int steps=atoi(c.c_str());
double step_size=(end-start)/(steps-1);

  for(int i=0; i<steps; i++){
    m_detection_dist.push_back(start);
    start=start+step_size;
    m_first_detection.push_back(true);
    int l=0;
    m_detection_total.push_back(l);
    m_detection_run.push_back(l);
    m_more_than_one.push_back(0);
    m_current_vname.push_back("");
  }
  return true;
}

bool SearchDetect::setTarget(string name)
{
  m_target_name=tolower(name);
  return true;
}
//------------------------------------------------------------
// Procedure: buildReport()
void SearchDetect::getEstimate()
{
  NodeRecord first = string2NodeRecord(m_first_report);
  NodeRecord second = string2NodeRecord(m_second_report);
  NodeRecord target = string2NodeRecord(m_target_report);
  double x1 = first.getX();
  double y1= first.getY();
  double t1= first.getTimeStamp();


  double x2 = second.getX();
  double y2= second.getY();
  double t2= second.getTimeStamp();

  double add_heading = target.getHeading();
  double add_speed = target.getSpeed();


  //calculate change in dist 
  double dx2=(x2-x1)*(x2-x1);
  double dy2=(y2-y1)*(y2-y1);
  double delta_d=sqrt(dx2+dy2);
  //calculate speed estimate from change in pos and change in time
  double d_time=abs(t2-t1);
  double speed_est=delta_d/d_time;
  if(.05<speed_est&&speed_est<3){
  Notify("SPEED_ESTIMATE",speed_est);
  Notify("TARGET_SPEED",add_speed);
  //get heading estimate
  double heading_est=0;

  double y=y2-y1;
  double x=x2-x1;
  heading_est=atan2 (y,x) * 180 / M_PI;
  

  heading_est=heading_est+90;
  Notify("HEADING_ESTIMATE",heading_est);
  Notify("TARGET_HEADING",add_heading);
  double h_error=abs((heading_est-add_heading)/add_heading);
  double s_error=abs((speed_est-add_speed)/add_speed);
  Notify("HEADING_ERROR",h_error);
  Notify("SPEED_ERROR",s_error);
  }
}
bool SearchDetect::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

m_msgs << "============================================" << endl;
m_msgs << "NUM_HITS = " << m_numhit<<endl;
m_msgs << "============================================" << endl;
m_msgs << "============================================" << endl;
m_msgs << "SIGMA = " << m_sigma<<endl;
m_msgs << "============================================" << endl;
m_msgs << "============================================" << endl;
m_msgs << "Probability = " << m_p<<endl;
m_msgs << "============================================" << endl;
m_msgs << "============================================" << endl;
m_msgs << "CPA = " << m_cpaunder<<endl;
m_msgs << "============================================" << endl;
m_msgs << "============================================" << endl;
m_msgs << "Rand Val = " << m_rand_val<<endl;
m_msgs << "============================================" << endl;
m_msgs << "============================================" << endl;
m_msgs << "First Report = " << m_first_report<<endl;
m_msgs << "============================================" << endl;
m_msgs << "============================================" << endl;
m_msgs << "Second Report = " << m_second_report<<endl;
m_msgs << "============================================" << endl;
m_msgs << "============================================" << endl;
m_msgs << "Target Report = " << m_target_report<<endl;
m_msgs << "============================================" << endl;
  return(true);
}