/*****************************************************************/
/*    Name: Raymond Turrisi                                      */
/*    Origin: Dept of Mechanical Engineering, MIT, Cambridge MA  */
/*    File: EvalEngine.cpp                                   */
/*    Developed: Summer 2023                                     */
/*    LastEd: June 5th, 2023                                     */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "EvalEngine.h"
#include <sstream>
#include <time.h>
using namespace std;

//---------------------------------------------------------
// Constructor()

EvalEngine::EvalEngine()
{
  m_engine_name = "EvalEngine";
}

//---------------------------------------------------------
// Destructor()

EvalEngine::~EvalEngine()
{
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    delete kernel_it->second;
  }
  m_eval_kernels.clear();
}

bool EvalEngine::setIdentity(std::string identity)
{
  m_engine_name = identity;
  return true;
}

//---------------------------------------------------------
// Procedure: Initialize()

bool EvalEngine::Initialize()
{
  // Now that we have all the parameters, simply let the kernels update themselves
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  dbg_print("<%s> Initializing\n", m_engine_name.c_str());
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    kernel_it->second->set_debug_stream(m_debug_stream_name);
    dbg_print("<%s> Set Params on: %s\n", m_engine_name.c_str(), kernel_it->second->m_identity.c_str());
    kernel_it->second->SetParams(m_parameters_all);
  }

  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    // Since we allow remappable subscriptions, after the parameters are updated, we then collect the final list of subscriptions
    std::set<std::string> kernel_subscriptions = kernel_it->second->get_subscriptions();
    std::set<std::string>::iterator sub_it;
    for (sub_it = kernel_subscriptions.begin(); sub_it != kernel_subscriptions.end(); sub_it++)
    {
      m_subscriptions_all.insert(*sub_it);
      m_subscriptions_to_kernels[*sub_it].insert(kernel_it->second->m_identity);
    }
  }

  // We don't register for variables if we are processing an alogfile that already exists

  return (true);
}

//---------------------------------------------------------
// Procedure: UpdateVariables()

bool EvalEngine::UpdateVariables(std::map<std::string, EEMessage> msg_buffer)
{

  std::map<std::string, EEMessage>::iterator p;

  // Unpack the collection of MOOS Messages within this timeframe of receiving new mail
  for (p = msg_buffer.begin(); p != msg_buffer.end(); p++)
  {
    string key = p->second.getVarName();
    m_info_buffer[key] = p->second;
  }

  // Now that we have all the mail since the last cycle, we distribute this mail to the kernels
  std::map<std::string, EEMessage>::iterator it_info;
  for (it_info = m_info_buffer.begin(); it_info != m_info_buffer.end(); ++it_info)
  {
    // For each topic, we go through all the kernels which have subscribed to this topic
    std::set<std::string>::iterator kernel_it;
    for (kernel_it = m_subscriptions_to_kernels[it_info->first].begin(); kernel_it != m_subscriptions_to_kernels[it_info->first].end(); ++kernel_it)
    {
      dbg_print("Updating: %s with topic %s\n", (*kernel_it).c_str(), it_info->first.c_str());
      bool successful_update = m_eval_kernels[*kernel_it]->UpdateVariable(m_info_buffer[it_info->first]);
      if (successful_update == false)
      {
        dbg_print("Unhandled Mail: Kernel <%s> -> Msg<key=%s,val=%s>\n", (*kernel_it).c_str(), it_info->first.c_str(), it_info->second.msg().c_str());
        // reportRunWarning("Unhandled Mail: Kernel <" + *kernel_it + "> -> Msg<key=" + it_info->first + ",val=" + it_info->second.msg() + ">");
      }
    }
  }

  // Clear all the new mail that we've received
  m_info_buffer.clear();

  return (true);
}

//---------------------------------------------------------
// Procedure: ComputeEvaluations()
//            happens AppTick times per second

std::vector<std::pair<std::string, std::string> > EvalEngine::ComputeEvaluations(std::vector<std::string> &kernel_level_warnings)
{
  // Let all the kernels receive new mail, do error checking, and update their variables
  // for all the updates we have, we go through the names of the topics which have new updates
  // Each kernel should be designed to handle one piece of mail at a time, in order to enable high-level error checking
  std::vector<std::pair<std::string, std::string> > updates;

  // Now go through all the kernels and let them provide their updates
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
    kernel_it->second->Evaluate(updates, kernel_level_warnings);

  if(kernel_level_warnings.size()) {
    std::vector<std::string>::iterator warning_it;
    dbg_print("%s - Runtime Warnings Present\n", m_engine_name.c_str());
    for(warning_it = kernel_level_warnings.begin(); warning_it != kernel_level_warnings.end(); ++warning_it) {
      dbg_print("\t%s\n", warning_it->c_str());
    }
  }

  return updates;
}

bool EvalEngine::setTime(double moos_time)
{
  m_info_buffer["MOOSTime"] = EEMessage("MOOSTime", moos_time, moos_time);
  return true;
}

//---------------------------------------------------------
// Procedure: setParameter()
bool EvalEngine::setParameter(std::string param, std::string value)
{
  //TODO: Maybe we can add a mechanism for debugging at the kernel level
  m_parameters_all[param] = value;
  return true;
}

//---------------------------------------------------------
// Procedure: getParameterKeys()
std::set<std::string> EvalEngine::getParameterKeys()
{
  std::set<std::string> param_keys;
  std::map<std::string, std::string>::iterator it;
  for (it = m_parameters_all.begin(); it != m_parameters_all.end(); ++it)
    param_keys.insert(it->first);
  return param_keys;
}

//---------------------------------------------------------
// Procedure: getSubscriptions()

std::set<std::string> EvalEngine::getSubscriptions()
{
  return m_subscriptions_all;
}

//---------------------------------------------------------
// Procedure: getPublications()

std::set<std::string> EvalEngine::getPublications()
{
  return m_publications_all;
}

//---------------------------------------------------------
// Procedure: insertKernel()

void EvalEngine::insertKernel(EvalEngineKernel *kernel)
{
  // We can traverse and navigate our collection of kernels with iterators and by looking them up
  // by their identity

  std::map<std::string, std::string> kernel_parameters = kernel->get_parameters();
  std::map<std::string, std::string>::iterator param_it;
  
  for (param_it = kernel_parameters.begin(); param_it != kernel_parameters.end(); param_it++)
    m_parameters_all[param_it->first] = param_it->second;

  m_eval_kernels[kernel->m_identity] = kernel;
}

// TODO
std::string EvalEngine::genEngineReleaseInfo()
{
  return "showReleaseInfoAndExit\n";
}

std::string EvalEngine::genEngineExampleConfig()
{

  stringstream ecp; // engine configuration parameters

  // For each kernel which is included is built into this EvalEngine, generate a configuration file template with default values
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    // Kernel name
    ecp << "//    > " << kernel_it->second->m_identity.c_str() << " (default values as shown)" << endl;
    // First find the longest parameter name, and format the width of each line accordinly so everything is nicely aligned
    std::map<std::string, std::string> parameters = kernel_it->second->get_parameters();
    uint16_t longest_name = 0;
    std::map<std::string, std::string>::iterator param_it;
    for (param_it = parameters.begin(); param_it != parameters.end(); ++param_it)
      if (param_it->first.size() > longest_name)
        longest_name = param_it->first.size();

    // If there are no parameters for a kernel, display such
    if (parameters.size() == 0)
      ecp << "//   ( no parameters)" << endl;

    // For each parameter if they exist, print out the parameter name and the default value
    for (param_it = parameters.begin(); param_it != parameters.end(); ++param_it)
      ecp << "        " << std::right << std::setw(longest_name) << param_it->first << " = " << param_it->second << endl;
  }
  return ecp.str();
}

// TODO
std::string EvalEngine::genEngineInterface()
{
  return "showInterfaceAndExit\n";
}

//---------------------------------------------------------
// set_debug_stream

void EvalEngine::set_debug_stream(char debug_stream_name[])
{
  m_debug_stream_name = debug_stream_name;
}

//---------------------------------------------------------
// dbg print

bool EvalEngine::dbg_print(const char *format, ...)
{
  if (m_debug_stream_name != nullptr)
  {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_debug_stream_name, "a");
    if (m_debug_stream != nullptr)
    {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      return true;
    }
    else
    {
      return false;
    }
  }
  return false;
}

//------------------------------------------------------------
// Procedure: buildReport()

bool EvalEngine::genAppCastingReport(std::stringstream &m_msgs)
{

  ACTable actab(2);

  actab << "Kernel | Report";
  actab.addHeaderLines();
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    kernel_it->second->AppCastingNotice(actab);
  }
  m_msgs << actab.getFormattedString();
  return (true);
}
