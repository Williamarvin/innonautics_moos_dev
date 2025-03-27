#--------------------------------------------------------
# The CMakeLists.txt for:                       uFldSearchDetect
# Author(s):                              Craig Evans
#--------------------------------------------------------

SET(SRC
  SearchDetect.cpp
  SearchDetect_Info.cpp
  CPAMonitor.cpp
  CPAEventNew.cpp
  main.cpp
)

ADD_EXECUTABLE(uFldSearchDetect ${SRC})

TARGET_LINK_LIBRARIES(uFldSearchDetect
   ${MOOS_LIBRARIES}
   apputil
   contacts
   geometry
   mbutil
   ufield
   encounters
   geometry
   logic
   pthread)

