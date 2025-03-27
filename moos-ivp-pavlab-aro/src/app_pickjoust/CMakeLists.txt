#--------------------------------------------------------
# The CMakeLists.txt for:                   app_pickjoust
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

FILE(GLOB SRC *.cpp)

ADD_EXECUTABLE(pickjoust ${SRC})

TARGET_LINK_LIBRARIES(pickjoust
  geometry
  mbutil
  ${SYSTEM_LIBS}
)
