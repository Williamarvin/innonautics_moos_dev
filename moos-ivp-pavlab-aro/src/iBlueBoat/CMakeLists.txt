#--------------------------------------------------------
# The CMakeLists.txt for:                       iBlueBoat
# Author(s):                              Jeremy Wenger
#--------------------------------------------------------

SET(SRC
  BlueBoat.cpp
  BlueBoat_Info.cpp
  Thruster.cpp
  main.cpp
)

ADD_EXECUTABLE(iBlueBoat ${SRC})

TARGET_LINK_LIBRARIES(iBlueBoat
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   sock_util
   mbutil
   m
   pthread
)
   
SET(SRC_GARB
testninjas.cpp
)

ADD_EXECUTABLE(testninjas ${SRC_GARB})

TARGET_LINK_LIBRARIES(testninjas
${MOOS_LIBRARIES}
geometry
apputil
sock_util
mbutil
m
pthread)


SET(SRC_TESTING
test_BlueBoatBridge.cpp
)

ADD_EXECUTABLE(test_BlueBoatBridge ${SRC_TESTING})

TARGET_LINK_LIBRARIES(test_BlueBoatBridge
${MOOS_LIBRARIES}
geometry
apputil
sock_util
mbutil
m
pthread)