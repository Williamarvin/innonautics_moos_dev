#--------------------------------------------------------
# The CMakeLists.txt for:             uFldCollisionDetect
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  SET(SYSTEM_LIBS
    wsock32)
else (${WIN32})
  SET(SYSTEM_LIBS
    m
    pthread)
endif (${WIN32})

SET(SRC
  CollisionDetector.cpp
  CollisionDetector_Info.cpp
  CPAMonitor.cpp
  CPAEvent_mnm.cpp
  EncounterReport.cpp
  main.cpp
)
 
ADD_EXECUTABLE(uFldColregsDetect ${SRC})

TARGET_LINK_LIBRARIES(uFldColregsDetect
   ${MOOS_LIBRARIES}
   apputil
   contacts
   geometry
   mbutil
   ufield
   logic
   ${SYSTEM_LIBS}
)
