#--------------------------------------------------------
# The CMakeLists.txt for:                 pDemusterAssign
# Author(s):                              Filip Stromstad
#--------------------------------------------------------

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

SET(SRC
  DemusterAssign.cpp
  DemusterAssign_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pDemusterAssign ${SRC})

TARGET_LINK_LIBRARIES(pDemusterAssign
   apputil
   contacts
   geometry
   mbutil
   pthread
   dubin
   demuster_geometry
   hungarian_ext
   ${MOOS_LIBRARIES}
   ${SYSTEM_LIBS})

