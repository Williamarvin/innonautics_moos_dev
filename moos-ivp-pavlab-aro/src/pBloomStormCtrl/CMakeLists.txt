#--------------------------------------------------------
# The CMakeLists.txt for:                       pBloomStormCtrl
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  BloomStormCtrl.cpp
  BloomStormCtrl_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBloomStormCtrl ${SRC})

TARGET_LINK_LIBRARIES(pBloomStormCtrl
  contacts
  geometry
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

