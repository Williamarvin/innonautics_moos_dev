#--------------------------------------------------------
# The CMakeLists.txt for:                       iSerial
# Author(s):                              Blake Cole
#--------------------------------------------------------

SET(SRC
  Serial.cpp
  Serial_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iSerial ${SRC})

TARGET_LINK_LIBRARIES(iSerial
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

