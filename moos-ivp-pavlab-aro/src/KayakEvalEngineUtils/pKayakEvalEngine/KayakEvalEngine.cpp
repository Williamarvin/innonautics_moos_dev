/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: KayakEvalEngine.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "KayakEvalEngine.h"
#include "std_ee_kernels.h"
using namespace std;

//---------------------------------------------------------
// Constructor()

KayakEvalEngine::KayakEvalEngine()
{
  //TODO: Add a suffix option, such that all the default parameter names are updated, so you can have multiple kernels with unique parameter names
  evalEngine.insertKernel(new xSurfaceOdometryKernel());
  evalEngine.insertKernel(new xCounterKernel("xCounter1", "_1"));
  evalEngine.insertKernel(new xCounterKernel("xCounter2", "_2"));
  evalEngine.insertKernel(new xStateTimer());
}

//---------------------------------------------------------
// Destructor

KayakEvalEngine::~KayakEvalEngine()
{
}

//---------------------------------------------------------
// dbg print

bool KayakEvalEngine::dbg_print(const char *format, ...)
{
  // wrap formatted print statements to go to stdout, stderr, or a file
  // if its a file, handle it cleanly
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    if (m_fname[0] == '\0')
    {
      vfprintf(m_debug_stream, format, args);
    }
    else
    {
      m_debug_stream = fopen(m_fname, "a");
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
    }
    return true;
  }
  return false;
}


//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool KayakEvalEngine::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  evalEngine.setIdentity(GetAppName());
  evalEngine.setTime(MOOSTime());

  evalEngineParameters = evalEngine.getParameterKeys();

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;

    if (param == "debug")
    {
      m_debug = (value == tolower("true")) ? true : false;
      
      if (m_debug)
      {
        time_t rawtime;
        struct tm *timeinfo;
        memset(m_fname, m_fname_buff_size, '\0');
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char fmt[m_fname_buff_size];
        memset(fmt, m_fname_buff_size, '\0');
        strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                 GetAppName().c_str(), fmt);
        evalEngine.set_debug_stream(m_fname);
      }
      handled = true;
    }
    else if (evalEngineParameters.count(param))
    {
      handled = evalEngine.setParameter(param, value);
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  evalEngine.Initialize();

  evalEngineSubscriptions = evalEngine.getSubscriptions();
  evalEnginePublications = evalEngine.getPublications();

  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool KayakEvalEngine::OnConnectToServer()
{

  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void KayakEvalEngine::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  std::set<std::string>::iterator it;
  for (it = evalEngineSubscriptions.begin(); it != evalEngineSubscriptions.end(); ++it)
  {
    Register(*it, 0);
  }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool KayakEvalEngine::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);
  evalEngine.setTime(MOOSTime());

  MOOSMSG_LIST::iterator p;
  std::map<std::string, EEMessage> evalEngineBuffer;

  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if (key == "FOO")
      cout << "great!";
    else if (evalEngineSubscriptions.count(key))
    {
      //TODO: Include a function which takes the CMOOSMail variable or eliminate EEMessage altogether. Just have to figure out the best way to handle time
      evalEngineBuffer[key] = EEMessage(msg);
    }

    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  evalEngine.UpdateVariables(evalEngineBuffer);

  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool KayakEvalEngine::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  evalEngine.setTime(MOOSTime());
  std::vector<std::string> kernel_level_warnings;
  //TODO: Will eventually update this type to include an auxiliary from which kernel it comes from
  std::vector<std::pair<std::string, std::string> > evalEngineUpdates = evalEngine.ComputeEvaluations(kernel_level_warnings);
  std::vector<std::pair<std::string, std::string> >::iterator it;
  
  for (it = evalEngineUpdates.begin(); it != evalEngineUpdates.end(); ++it)
    Notify(it->first, it->second);
  if(kernel_level_warnings.size()) {
    std::vector<std::string>::iterator warning_it;
    for (warning_it = kernel_level_warnings.begin(); warning_it != kernel_level_warnings.end(); ++warning_it)
      reportRunWarning(*warning_it);
  }
  AppCastingMOOSApp::PostReport();
  return (true);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool KayakEvalEngine::buildReport()
{
  // TODO: Build high-level EvalEngine style report before moving onto kernel reports
  m_msgs << "============================================" << endl;
  m_msgs << "File: pKayakEvalEngine Build Report         " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(2);
  actab << "pKayakEvalEngine | Notice";
  actab.addHeaderLines();
  actab << "Status" << "No Process Level Notices";
  m_msgs << actab.getFormattedString();

  evalEngine.genAppCastingReport(m_msgs);

  return (true);
}

void KayakEvalEngine::showReleaseInfoAndExit() {
  cout << "showReleaseInfoAndExit" << endl;
  exit(0);
}

void KayakEvalEngine::showExampleConfigAndExit() {
  //Print the high level MOOS App configuration parameters
  cout << "//--------------------------------------------------------" << endl;
  cout << "// pKayakEvalEngine Example MOOS Configuration" << endl
       << endl;
  cout << "ProcessConfig = pKayakEvalEngine" << endl;
  cout << "{" << endl
       << endl;
  cout << "// Engine Configuration Parameters" << endl;
  cout << "      AppTick = 10" << endl;
  cout << "    CommsTick = 10" << endl
       << endl;
  cout << "        debug = false" << endl
       << endl;
  cout << "// Kernel-specific configuration parameters" << endl << endl;
  cout << evalEngine.genEngineExampleConfig() << endl;
  cout << "}" << endl;
  exit(0);
}

void KayakEvalEngine::showHelpAndExit() {
  cout << "showHelpAndExit" << endl;
  exit(0);
}

void KayakEvalEngine::showInterfaceAndExit() {
  cout << "showInterfaceAndExit" << endl;
  exit(0);
}