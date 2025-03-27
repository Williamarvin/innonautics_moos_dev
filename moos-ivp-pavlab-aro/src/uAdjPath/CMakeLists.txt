#--------------------------------------------------------
# The CMakeLists.txt for:                       uAdjPath
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  AdjPath.cpp
  AdjPath_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uAdjPath ${SRC})

TARGET_LINK_LIBRARIES(uAdjPath
  ${MOOS_LIBRARIES}
  geometry
  apputil
  mbutil
  m
  pthread)

