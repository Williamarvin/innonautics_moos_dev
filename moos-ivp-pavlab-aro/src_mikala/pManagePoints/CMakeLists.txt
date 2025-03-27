#--------------------------------------------------------
# The CMakeLists.txt for:                       pSendPoints
# Author(s):                              Mikala Molina
#--------------------------------------------------------

SET(SRC
  SendPoints.cpp
  SendPoints_Info.cpp
  VisitPoint.cpp
  NodeReport.cpp
  main.cpp
)

ADD_EXECUTABLE(pManagePoints ${SRC})

TARGET_LINK_LIBRARIES(pManagePoints
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)

