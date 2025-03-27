#--------------------------------------------------------
# The CMakeLists.txt for:            pDynamicTrafficLight
# Author(s):                              Filip Stromstad
#--------------------------------------------------------

# enable C++11 (see top-level CMakeLists.txt for macro definition)
use_cxx11()

SET(SRC
  DynamicTrafficLight.cpp
  DynamicTrafficLight_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pDynamicTrafficLight ${SRC})

TARGET_LINK_LIBRARIES(pDynamicTrafficLight
   ${MOOS_LIBRARIES}
   apputil
   geometry
   mbutil
   m
   pthread)

