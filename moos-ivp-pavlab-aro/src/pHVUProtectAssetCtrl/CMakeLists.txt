#--------------------------------------------------------
# The CMakeLists.txt for:            HVUProtectAssetCtrl
# Author(s):                              Tyler Paine
#--------------------------------------------------------

find_library(HELMTASK_LIBRARY NAMES helmtask PATHS ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/lib NO_DEFAULT_PATH)

if(NOT HELMTASK_LIBRARY)
  #TODO: Can make it so this app doesn't build?
  message(FATAL_ERROR "helmtask library not found")
endif()

SET(SRC
  HVUProtectAssetCtrl.cpp
  HVUProtectAssetCtrl_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pHVUProtectAssetCtrl ${SRC})

TARGET_INCLUDE_DIRECTORIES(pHVUProtectAssetCtrl PRIVATE 
    ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/src/lib_helmtask
)

TARGET_LINK_LIBRARIES(pHVUProtectAssetCtrl
  ${HELMTASK_LIBRARY}
  ${MOOS_LIBRARIES}
  ${MOOSGeodesy_LIBRARIES}
  apputil
  contacts
  geometry
  mbutil
  m
  pthread)

