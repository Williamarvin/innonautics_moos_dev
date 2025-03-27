#--------------------------------------------------------
# The CMakeLists.txt for:                       pBathyPath
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  BathyPath.cpp
  BathyPath_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBathyPath ${SRC})

TARGET_LINK_LIBRARIES(pBathyPath
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

