/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ThrusterAlloc.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ThrusterAlloc_HEADER
#define ThrusterAlloc_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "Thruster.h"

class ThrusterAlloc : public AppCastingMOOSApp
{
 public:
   ThrusterAlloc();
   ~ThrusterAlloc();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
   Thruster m_thruster;

   double m_rud_time;
   double m_thr_time;
   double m_stale_thresh; 
   
   
};

#endif 
