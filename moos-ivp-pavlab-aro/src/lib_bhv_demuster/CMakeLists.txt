# ------------------------------------------------------------------------------
# The CMakeLists.txt for:            lib_bhv_demuster Author(s): Filip Stromstad
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
#                                            BHV_Demuster
#--------------------------------------------------------
ADD_LIBRARY(BHV_Demuster SHARED 
BHV_Demuster.cpp)
TARGET_LINK_LIBRARIES(BHV_Demuster
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