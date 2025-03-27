#--------------------------------------------------------
# The CMakeLists.txt for:                       pProxonoiGridSearch
# Author(s):                              Tyler Paine
#--------------------------------------------------------
find_library(VORONOI_LIBRARY NAMES voronoi PATHS ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/lib NO_DEFAULT_PATH)

if(NOT VORONOI_LIBRARY)
  #TODO: Can make it so this app doesn't build?
  message(FATAL_ERROR "voronoi library not found")
endif()



SET(SRC
  ProxonoiGridSearch.cpp
  ProxonoiGridSearch_Info.cpp
  ProxGrid.cpp
  main.cpp
)

ADD_EXECUTABLE(pProxonoiGridSearch ${SRC})

TARGET_INCLUDE_DIRECTORIES(pProxonoiGridSearch PRIVATE 
    ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/src/lib_voronoi
)


TARGET_LINK_LIBRARIES(pProxonoiGridSearch
  ${VORONOI_LIBRARY}
  ${MOOS_LIBRARIES}
  contacts
   geometry
   apputil
   mbutil
   m
   pthread
   path_plan)

