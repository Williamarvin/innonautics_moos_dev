#--------------------------------------------------------
# The CMakeLists.txt for:                       iPingSim
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  PingSim.cpp
  PingSim_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iPingSim ${SRC})

TARGET_LINK_LIBRARIES(iPingSim
   ${MOOS_LIBRARIES}
   MOOSGeodesy
   geometry
   apputil
   mbutil
   m
   pthread
   bathygrid
)

