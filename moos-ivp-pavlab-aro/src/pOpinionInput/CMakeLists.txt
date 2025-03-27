#--------------------------------------------------------
# The CMakeLists.txt for:                       pOpinionInput
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  OpinionInput.cpp
  OpinionInput_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pOpinionInput ${SRC})

TARGET_LINK_LIBRARIES(pOpinionInput
  ${MOOS_LIBRARIES}
  opinion
  geometry
  apputil
  mbutil
  m
  pthread)

