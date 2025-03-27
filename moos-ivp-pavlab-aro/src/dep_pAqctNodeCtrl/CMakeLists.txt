#--------------------------------------------------------
# The CMakeLists.txt for:                       pAqctNodeCtrl
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  AqctNodeCtrl.cpp
  AqctNodeCtrl_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pAqctNodeCtrl ${SRC})

TARGET_LINK_LIBRARIES(pAqctNodeCtrl
  ${MOOS_LIBRARIES}
  contacts
  geometry
  apputil
  mbutil
  m
  pthread)

