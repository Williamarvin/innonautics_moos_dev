#--------------------------------------------------------
# The CMakeLists.txt for:                       pOpinionManager
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  OpinionManager.cpp
  OpinionManager_Info.cpp
  main.cpp
  OpinionManagerEngine.cpp
)

ADD_EXECUTABLE(pOpinionManager ${SRC})

TARGET_LINK_LIBRARIES(pOpinionManager
  ${MOOS_LIBRARIES}
  opinion
  apputil
  mbutil
  logic
  m
  pthread)

