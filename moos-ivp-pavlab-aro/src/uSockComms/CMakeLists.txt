#--------------------------------------------------------
# The CMakeLists.txt for:                      uSockComms
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  SockComms.cpp
  SockComms_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uSockComms ${SRC})

TARGET_LINK_LIBRARIES(uSockComms
   ${MOOS_LIBRARIES}
   apputil
   sock_util
   mbutil
   m
   pthread)

