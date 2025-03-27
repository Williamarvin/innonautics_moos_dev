#--------------------------------------------------------
# The CMakeLists.txt for:            lib_bhv_tasks_dubins
# Author(s):                              Raymond Turrisi     
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})

find_library(HELMTASK_LIBRARY NAMES helmtask PATHS ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/lib NO_DEFAULT_PATH)
  
if(NOT HELMTASK_LIBRARY)
  #TODO: Can make it so this app doesn't build?
  message(FATAL_ERROR "helmtask library not found")
endif()

#--------------------------------------------------------
#                                          BHV_TaskConvoy3
#--------------------------------------------------------
ADD_LIBRARY(BHV_TaskConvoy3 SHARED 
  BHV_TaskConvoy3.cpp )
  target_include_directories(BHV_TaskConvoy3 PRIVATE ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/src/lib_helmtask)

  TARGET_LINK_LIBRARIES(BHV_TaskConvoy3
  ${HELMTASK_LIBRARY}
  helmivp
  contacts
  behaviors
  ivpbuild 
  logic
  ivpcore 
  bhvutil
  mbutil 
  geometry 
  ${SYSTEM_LIBS} )

#--------------------------------------------------------
#                                       BHV_TaskWaypoint3
#--------------------------------------------------------
ADD_LIBRARY(BHV_TaskWaypoint3 SHARED 
  BHV_TaskWaypoint3.cpp )
  target_include_directories(BHV_TaskWaypoint3 PRIVATE ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/src/lib_helmtask)

  TARGET_LINK_LIBRARIES(BHV_TaskWaypoint3
  ${HELMTASK_LIBRARY}
  helmivp
  contacts
  behaviors
  ivpbuild 
  logic
  ivpcore 
  bhvutil
  mbutil 
  geometry 
  ${SYSTEM_LIBS} )

