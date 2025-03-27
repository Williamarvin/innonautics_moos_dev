#--------------------------------------------------------
# The CMakeLists.txt for:                       uSimHeron
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

# Set System Specific Libraries
SET(SYSTEM_LIBS m pthread )

FILE(GLOB SRC *.cpp)

ADD_EXECUTABLE(uSimHeron ${SRC})
   
TARGET_LINK_LIBRARIES(uSimHeron
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   sock_util
   mbutil
   ${SYSTEM_LIBS} )

message("+++++++++++++++++++++++++++++++++++++++++")
message("uSimM300 SYSTEM_LIBS:" ${SYSTEM_LIBS})
message("+++++++++++++++++++++++++++++++++++++++++")
