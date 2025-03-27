#--------------------------------------------------------
# The CMakeLists.txt for:            lib_bhv_task_formations
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
#                                          BHV_TaskFormationFollower
#--------------------------------------------------------
ADD_LIBRARY(BHV_TaskFormationFollower SHARED 
  BHV_TaskFormationFollower.cpp )
  target_include_directories(BHV_TaskFormationFollower PRIVATE 
    ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/src/lib_helmtask
    ${CMAKE_SOURCE_DIR}/../moos-ivp-pavlab/src/lib_dubin
    )

  TARGET_LINK_LIBRARIES(BHV_TaskFormationFollower
  ${HELMTASK_LIBRARY}
  dubin
  helmivp
  contacts
  behaviors
  ivpbuild 
  logic
  dubin
  ivpcore 
  bhvutil
  mbutil 
  geometry 
  ${SYSTEM_LIBS} )

#--------------------------------------------------------
#                                       BHV_TaskFormationLeader
#--------------------------------------------------------
ADD_LIBRARY(BHV_TaskFormationLeader SHARED 
BHV_TaskFormationLeader.cpp )
  target_include_directories(BHV_TaskFormationLeader PRIVATE 
    ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/src/lib_helmtask
    ${CMAKE_SOURCE_DIR}/../moos-ivp-pavlab/src/lib_dubin
    )

  TARGET_LINK_LIBRARIES(BHV_TaskFormationLeader
  ${HELMTASK_LIBRARY}
  dubin
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

