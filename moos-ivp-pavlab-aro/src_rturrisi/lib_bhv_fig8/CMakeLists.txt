#--------------------------------------------------------
# The CMakeLists.txt for:           
# Author(s):                                  Ray Turrisi
#--------------------------------------------------------

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
  #                                       BHV_Fig8
  #--------------------------------------------------------

  ######### Behavior
  ADD_LIBRARY(BHV_Fig8 SHARED 
    BHV_Fig8.cpp
  )

  TARGET_LINK_LIBRARIES(BHV_Fig8
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

