#--------------------------------------------------------
# The CMakeLists.txt for:           
# Author(s):                                  Ray Turrisi
#--------------------------------------------------------

if(ARMA_AVAIL)

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
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

  #--------------------------------------------------------
  #                                       BHV_FormationCtrl
  #--------------------------------------------------------

  ######### Behavior
  ADD_LIBRARY(BHV_FormationCtrl SHARED 
    AgentStateInfo.cpp
    BHV_FormationCtrl.cpp
    CtrlPointQueue.cpp
  )

  target_include_directories(BHV_FormationCtrl PRIVATE 
    ${ARMADILLO_INCLUDE_DIR}
  )

  TARGET_LINK_LIBRARIES(BHV_FormationCtrl
  ${ARMA_LIBS}
    mbutil 
    helmivp
    behaviors
    geometry
    contacts
    ivpbuild 
    logic 
    ivpcore 
    bhvutil 
    geometry 
    ${MOOS_LIBRARIES}
    ${SYSTEM_LIBS} 
  )

  ######### Test application 
  SET(SRC
    test_formation_geometry.cpp
    AgentStateInfo.cpp
    CtrlPointQueue.cpp
  )

  ADD_EXECUTABLE(test_formation_geometry ${SRC})

  target_include_directories(test_formation_geometry PRIVATE 
    ${ARMADILLO_INCLUDE_DIR})

  TARGET_LINK_LIBRARIES(test_formation_geometry
  ${ARMA_LIBS}  
  ${MOOS_LIBRARIES}  
    geometry
    apputil
    mbutil
    m
    pthread)

else()
   message(WARNING "Armadillo dependencies not found - Not Building <pNavigator_AHRS>")
endif()

