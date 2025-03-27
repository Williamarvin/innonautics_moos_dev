/************************************************************/
/*    NAME: Supun Randeni                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ChangeMyName.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ChangeMyName.h"

using namespace std;

//---------------------------------------------------------
// Constructor

ChangeMyName::ChangeMyName()
{
}

//---------------------------------------------------------
// Destructor

ChangeMyName::~ChangeMyName()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool ChangeMyName::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    std::string sval  = msg.GetString();
    double dval = msg.GetDouble();

    // Loop is for all configured trigger variables
    for (int n=0; n<config.size(); n++)
    {
      CONFIG *buf = &config[n];

      // Check if we got the IN var of this field
      if (key == buf->in)
      {
        // Check if the content of IN msg is double or string, and add to buffer
        if(msg.IsDouble()){
          in_msg_double[n] = dval;
          in_msg_isdouble[n] = true;
          in_msg_sent[n] = false;
        }
        else if(msg.IsString()){
          in_msg_string[n] = sval;
          in_msg_isdouble[n] = false;
          in_msg_sent[n] = false;
        }

        // Is this IN msg triggered by bool
        if(triggered[n] == true && buf->bool_trigger == true){
          if (in_msg_isdouble[n])
            Notify(buf->out, in_msg_double[n]);
          else
            Notify(buf->out, in_msg_string[n]);
          trigger_time[n] = MOOSTime();
          in_msg_sent[n] = true;
        }
      }

      // Check if we got the trigger var of this field
      if (key == buf->trigger)
      {
        if (buf->bool_trigger == true){
          bool trigger_ = false;
          setBooleanOnString(trigger_, sval);
          if (trigger_==buf->bool_trigger_flag){
            triggered[n] = true;
          }
          else{
            triggered[n] = false;
          }
        }
        else if (!in_msg_sent[n]) {
          if (in_msg_isdouble[n])
            Notify(buf->out, in_msg_double[n]);
          else
            Notify(buf->out, in_msg_string[n]);
          in_msg_sent[n] = true;
          trigger_time[n] = MOOSTime();
          triggered[n] = true;
        }
      }

    }
  }
  
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool ChangeMyName::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ChangeMyName::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ChangeMyName::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  int n = 0;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    n += 1;
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "trigger") {
      string _s;
      CONFIG config_buf;
      if(MOOSValFromString(_s,orig,"trigger"))
      {
        std::cout<< n <<". Trigger = " << _s.c_str() << std::endl;
        config_buf.trigger = _s;  

        if(MOOSValFromString(_s,orig,"in"))
        {
          std::cout<< n <<". In = " << _s.c_str() << std::endl;
          config_buf.in = _s;  

          if(MOOSValFromString(_s,orig,"out"))
          {
            std::cout<< n <<". Out = " << _s.c_str() << std::endl;
            config_buf.out = _s;

              if(MOOSValFromString(_s,orig,"trigger_type"))
              {
                bool trigger_flag_=false;
                if (!setBooleanOnString(trigger_flag_, _s))
                {
                  std::cout<< n <<". Trigger type (on receipt)= " << _s.c_str() << std::endl;
                  config_buf.bool_trigger = false;
                  config_buf.bool_trigger_flag = false;
                }
                else
                {
                  std::cout<< n <<". Trigger type (bool)= " << _s.c_str() << std::endl;
                  config_buf.bool_trigger = true;
                  config_buf.bool_trigger_flag = trigger_flag_;
                }
                
                config.push_back(config_buf);
                handled = true;      
              }        
          }
        }        
      }
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();  
  in_msg_string.resize(config.size());
  in_msg_double.resize(config.size());
  in_msg_isdouble.resize(config.size());
  in_msg_sent.resize(config.size());
  trigger_time.resize(config.size(),-1);
  triggered.resize(config.size(),false);
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void ChangeMyName::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  for (int n=0; n<config.size(); n++)
    {
      CONFIG *b = &config[n];
      // std::string register_var = b->in;
      Register(b->in, 0);
      Register(b->trigger, 0);
      // std::cout <<"Registering variable= " << b->in << std::endl;
    }
}


//------------------------------------------------------------
// Procedure: buildReport()

bool ChangeMyName::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: uChangeMyName                         " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(5);
  actab << "Trigger var | Last trigger time | Trigger flag | Ori var | New var ";
  actab.addHeaderLines();
  for (int n=0; n<config.size(); n++)
    {
      CONFIG *b = &config[n];
      actab << b->trigger << trigger_time[n] << triggered[n] << b->in << b->out;
    }
  
  m_msgs << actab.getFormattedString();

  return(true);
}




