#--------------------------------------------------------
# The CMakeLists.txt for:                       pRoutePlan
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  RoutePlan.cpp
  RoutePlan_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pRoutePlan ${SRC})

TARGET_LINK_LIBRARIES(pRoutePlan
   ${MOOS_LIBRARIES}
   MOOSGeodesy
   geometry
   apputil
   mbutil
   m
   pthread
   bathygrid
   path_plan
)

