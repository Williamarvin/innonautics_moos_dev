#--------------------------------------------------------
# The CMakeLists.txt for:                           iVehX
# Author(s):   Tyler Paine, Mike Defilippo, Mike Benjamin                               
#--------------------------------------------------------

SET(SRC
  VehX.cpp
  VehX_Info.cpp
  Thruster.cpp
  main.cpp
)

ADD_EXECUTABLE(iVehX ${SRC})

TARGET_LINK_LIBRARIES(iVehX
  ${MOOS_LIBRARIES}
  ${MOOSGeodesy_LIBRARIES}
  geometry
  apputil
  sock_util
  mbutil
  m
  pthread)

