#--------------------------------------------------------
# The CMakeLists.txt for:                       pPingProfileDetect
# Author(s):                              Craig Evans
#--------------------------------------------------------

SET(SRC
  PingProfileDetect.cpp
  PingProfileDetect_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pPingProfileDetect ${SRC})

TARGET_LINK_LIBRARIES(pPingProfileDetect
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

