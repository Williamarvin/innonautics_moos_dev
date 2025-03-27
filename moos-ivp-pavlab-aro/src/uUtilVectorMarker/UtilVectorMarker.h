/************************************************************/
/*    NAME: Tyler Paine                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: UtilVectorMarker.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef UtilVectorMarker_HEADER
#define UtilVectorMarker_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include "XYVector.h"
#include "XYPoint.h"
#include "XYFormatUtilsPoint.h"
#include "MBUtils.h"
#include "AngleUtils.h"

class UtilVectorMarker : public AppCastingMOOSApp
{
 public:
   UtilVectorMarker();
   ~UtilVectorMarker();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool handleVectorSet(std::string spec);
   void handleNewInput(std::string key, double val); 

 private: // Configuration variables
   std::string m_vname;
   std::string m_vis_cond_var;
   bool m_post_visuals;
   bool m_posted_new_visuals; 
   
   std::set<std::string> m_input_vars;
   double m_head_size;
   double m_max_mag;
   double m_vect_length_scale; 

 private: // State variables
   double m_nav_x;
   double m_nav_y;

   // The key is the variable name for the vector
   std::map<std::string, XYVector> m_vec_map;
   std::map<std::string, XYPoint>  m_point_map;  // where the vector points to
                                                 // which is updated
     
   
};

#endif 
