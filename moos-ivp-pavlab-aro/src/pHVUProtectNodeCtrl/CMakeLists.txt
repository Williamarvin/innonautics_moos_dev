#--------------------------------------------------------
# The CMakeLists.txt for:              pHVUProtectNodeCtrl
# Author(s):                              Tyler Paine
#--------------------------------------------------------

find_library(HELMTASK_LIBRARY NAMES helmtask PATHS ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/lib NO_DEFAULT_PATH)

if(NOT HELMTASK_LIBRARY)
  #TODO: Can make it so this app doesn't build?
  message(FATAL_ERROR "helmtask library not found")
endif()

SET(SRC
  HVUProtectNodeCtrl.cpp
  HVUProtectNodeCtrl_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pHVUProtectNodeCtrl ${SRC})

TARGET_INCLUDE_DIRECTORIES(pHVUProtectNodeCtrl PRIVATE 
    ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/src/lib_helmtask
)

TARGET_LINK_LIBRARIES(pHVUProtectNodeCtrl
  ${HELMTASK_LIBRARY}
  ${MOOS_LIBRARIES}
  ${MOOSGeodesy_LIBRARIES}
  apputil
  contacts
  geometry
  mbutil
  m
  pthread)

