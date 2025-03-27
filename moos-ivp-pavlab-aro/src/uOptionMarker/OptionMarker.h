/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OptionMarker.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef OptionMarker_HEADER
#define OptionMarker_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYCircle.h"
#include "OpinionRecord.h"

class OptionMarker : public AppCastingMOOSApp
{
 public:
   OptionMarker();
   ~OptionMarker();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();
   bool handleColorSpec(std::string spec);

 protected:
   void registerVariables();

 private: // Configuration variables
   double m_opinion_saturated_limit;
   bool m_post_circ_with_stale_opinion;

 private: // State variables

   OpinionRecord m_own_opinion_rec;
   XYCircle m_marker_circle;
   std::string m_last_spec_posted;
   
   bool m_update;
   bool m_post_visuals;
   bool m_posted_new_visuals;
   std::string m_vis_cond_var; 
   
   // key is option, value is color; 
   std::map<std::string, std::string> m_option_color_map; 
};

#endif 
