#--------------------------------------------------------
# The CMakeLists.txt for:                       alogceval
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

SET(SRC main.cpp LogEvalHandler.cpp)

ADD_EXECUTABLE(alogceval ${SRC})
   
TARGET_LINK_LIBRARIES(alogceval
  convoy
  apputil
  mbutil
  logutils
  ${SYSTEM_LIBS})

