/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: KayakEvalEngine.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef KayakEvalEngine_HEADER
#define KayakEvalEngine_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <cstdarg> //va_list, va_start, va_end
#include "EvalEngine.h"

class KayakEvalEngine : public AppCastingMOOSApp
{
public:
  KayakEvalEngine();
  ~KayakEvalEngine();

protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  void registerVariables();

protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

public: // Specialized integration for an EvalEngine-style app
  void showReleaseInfoAndExit();
  void showExampleConfigAndExit();
  void showHelpAndExit();
  void showInterfaceAndExit();

protected: // App-Specific functions
  bool dbg_print(const char *format, ...);

private:   // Configuration variables
  EvalEngine evalEngine;
  std::set<std::string> evalEngineParameters;
  std::set<std::string> evalEngineSubscriptions;
  std::set<std::string> evalEnginePublications;
  
private: // State variables
  bool m_debug;
  FILE *m_debug_stream;
  static const uint16_t m_fname_buff_size = 255;
  char m_fname[m_fname_buff_size];
};

#endif
