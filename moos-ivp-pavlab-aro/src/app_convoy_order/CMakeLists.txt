#--------------------------------------------------------
# The CMakeLists.txt for:                    convoy_order
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  SET(SYSTEM_LIBS
    wsock32)
else (${WIN32})
  SET(SYSTEM_LIBS
    m)
endif (${WIN32})

SET(SRC main.cpp OrderHandler.cpp)

ADD_EXECUTABLE(convoy_order ${SRC})
   
TARGET_LINK_LIBRARIES(convoy_order
  convoy
  mbutil
  ${SYSTEM_LIBS})

