/*****************************************************************/
/*    Name: Raymond Turrisi                                      */
/*    Origin: Dept of Mechanical Engineering, MIT, Cambridge MA  */
/*    File: EvalEngine.h                                     */
/*    Developed: Summer 2023                                     */
/*    LastEd: June 5th, 2023                                     */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#pragma once

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <cstdarg> //va_list, va_start, va_end
#include <set>
#include <map>
#include "ACTable.h"
#include "EvalEngineMessage.h"
#include "EvalEngineKernelCore.h"

class EvalEngine
{
public:
  EvalEngine();
  ~EvalEngine();

  bool setIdentity(std::string identity);
  void set_debug_stream(char debug_stream_name[]);

  bool Initialize();
  bool UpdateVariables(std::map<std::string, EEMessage> msg_buffer);
  std::vector<std::pair<std::string, std::string> > ComputeEvaluations(std::vector<std::string> &kernel_level_warnings);

  std::string genEngineReleaseInfo();
  std::string genEngineExampleConfig();
  std::string genEngineInterface();

  void insertKernel(EvalEngineKernel *kernel);

  bool genAppCastingReport(std::stringstream &m_msgs);

  bool setParameter(std::string, std::string);
  bool setTime(double moos_time);
  std::set<std::string> getParameterKeys();
  std::set<std::string> getSubscriptions();
  std::set<std::string> getPublications();

protected: // Protected methods
  bool dbg_print(const char *format, ...);

protected: // Protected variables
  std::map<std::string, EvalEngineKernel *> m_eval_kernels;

private: // Configuration variables
  std::set<std::string> m_subscriptions_all;
  std::set<std::string> m_publications_all;
  std::map<std::string, std::set<std::string> > m_subscriptions_to_kernels;
  std::map<std::string, std::set<std::string> > m_publications_to_kernels;
  std::vector<std::string> m_kernel_identities;
  std::map<std::string, EEMessage> m_info_buffer;
  std::map<std::string, std::string> m_parameters_all;
  std::map<std::string, std::string> m_updates_all;

private: // State variables
  std::string m_engine_name;
  bool m_debug;
  FILE *m_debug_stream;
  static const uint16_t m_fname_buff_size = 255;
  char *m_debug_stream_name;
};