#--------------------------------------------------------
# The CMakeLists.txt for:                       pMultiAgentColFilt
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  MultiAgentColFilt.cpp
  MultiAgentColFilt_Info.cpp
  main.cpp
  ForwardSetEst.cpp
)

ADD_EXECUTABLE(pMultiAgentColFilt ${SRC})

TARGET_LINK_LIBRARIES(pMultiAgentColFilt
  ${MOOS_LIBRARIES}
  contacts
  geometry
  apputil
  geometry
  mbutil
  m
  pthread)

