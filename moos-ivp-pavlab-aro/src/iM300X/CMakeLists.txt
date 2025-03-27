#--------------------------------------------------------
# The CMakeLists.txt for:                           iM300
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  M300.cpp
  M300_Info.cpp
  Thruster.cpp
  main.cpp
  VehRotController.cpp
)

ADD_EXECUTABLE(iM300X ${SRC})

TARGET_LINK_LIBRARIES(iM300X
  ${MOOS_LIBRARIES}
  ${MOOSGeodesy_LIBRARIES}
  geometry
  apputil
  sock_util
  mbutil
  m
  pthread)

