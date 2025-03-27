#--------------------------------------------------------
# The CMakeLists.txt for:                       pRequestPoints
# Author(s):                              Mikala Molina
#--------------------------------------------------------

SET(SRC
  RequestPoints.cpp
  RequestPoints_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pRequestPoints ${SRC})

TARGET_LINK_LIBRARIES(pRequestPoints
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   geometry
   pthread)

