#--------------------------------------------------------
# The CMakeLists.txt for:                   lib_sock_util
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  SockNinja.cpp
  SockNinjaProtoBuf.cpp
)

SET(HEADERS
  SockNinja.h
  SockNinjaProtoBuf.h
)

# Build Library
ADD_LIBRARY(sock_util ${SRC})

