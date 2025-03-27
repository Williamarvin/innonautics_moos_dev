# ------------------------------------------------------------------------------
# The CMakeLists.txt for:               lib_bhv_dubin Author(s): Filip Stromstad
# ------------------------------------------------------------------------------

# enable C++11 (see top-level CMakeLists.txt for macro definition)
use_cxx11()

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})

#--------------------------------------------------------
#                                          BHV_DubinsPath
#--------------------------------------------------------
ADD_LIBRARY(BHV_DubinsPath SHARED 
BHV_DubinsPath.cpp)
TARGET_LINK_LIBRARIES(BHV_DubinsPath
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   dubin
   geometry 
   ${MOOS_LIBRARIES}
   ${SYSTEM_LIBS} )