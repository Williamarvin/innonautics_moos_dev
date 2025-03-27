/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SumVars.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SumVars.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

SumVars::SumVars()
{
}

//---------------------------------------------------------
// Destructor

SumVars::~SumVars()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool SumVars::OnNewMail(MOOSMSG_LIST &NewMail)
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

    if((m_vals_map.count(key) > 0) && msg.IsDouble()){
      m_vals_map[key] = msg.GetDouble();
      m_got_vals_map[key] = true; 

    }else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool SumVars::OnConnectToServer()
{
  //registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SumVars::Iterate()
{
  AppCastingMOOSApp::Iterate();

  std::map<std::string, std::set<std::string> >::iterator it;
  for (it = m_algebra_map.begin(); it != m_algebra_map.end(); it++){
    // check that we have all the vars and sum them up
    double val = 0.0;

    if (m_operation_map[it->first] == "multiply"){
      val = 1.0;
    }
    
    std::set<std::string> inputs = it->second;  // for readabilty

    std::set<std::string>::iterator it2;
    for (it2 = inputs.begin(); it2 != inputs.end(); it2++){
      if (m_got_vals_map.count(*it2)>0){  // check if got key,
	                                  // both maps should have the same keys
	
	if (m_got_vals_map[*it2]){ // if got argument

	  if(m_operation_map[it->first] == "add"){
	    val += m_vals_map[*it2];
	  } else if (m_operation_map[it->first] == "multiply"){
	    val *= m_vals_map[*it2];
	  }
	  
	} else
	  continue;
      } else
	continue;
    }
    // If here we got all inputs, so ship it. 
    Notify(it->first, val);   
  }
  
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SumVars::OnStartUp()
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
    if(param == "sum_vars") {
      handled = handleAlgebraSpec(value, "add"); 
    }
    else if(param == "multi_vars") {
      handled = handleAlgebraSpec(value, "multiply"); 
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void SumVars::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  std::map<std::string, double>::iterator it;
  for (it = m_vals_map.begin(); it != m_vals_map.end(); it++){
    Register(it->first, 0);
  }
}


//------------------------------------------------------------
// Procedure: handleAlgebra spec

bool SumVars::handleAlgebraSpec(std::string value, std::string operation) 
{
  bool handled = false;
  // Get the output var
  std::string output_var = toupper(biteStringX(value, ':'));
  
  // Get the input vars
  std::set<std::string> new_set; 
  std::vector<std::string> inputs = parseString(value, ',');
  for (int i = 0; i < inputs.size(); i++){
    if (inputs[i] != ""){
      new_set.insert(toupper(inputs[i]));
    }
  }
  if ((output_var != "") && (new_set.size() > 0)){
    // all good.
    m_algebra_map[output_var] = new_set;
    
    // Add to the vals map with initial val of 0
    // and false 
    std::set<std::string>::iterator it;
    for (it = new_set.begin(); it != new_set.end(); it++){
      m_vals_map[*it] = 0.0;
      m_got_vals_map[*it] = false;
    }

    // record the operation
    if ((operation == "add") || (operation == "multiply")){
      
      m_operation_map[output_var] = operation;
      handled = true;
    }
  } 

  return(handled); 
}




//------------------------------------------------------------
// Procedure: buildReport()

bool SumVars::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




