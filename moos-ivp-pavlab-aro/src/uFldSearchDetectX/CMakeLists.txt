#--------------------------------------------------------
# The CMakeLists.txt for:               uFldSearchDetectX
# Author(s):              M.Benjamin mod from Craig Evans
#--------------------------------------------------------

SET(SRC
  SearchDetect.cpp
  SearchDetect_Info.cpp
  CPAMonitor.cpp
  CPAEventNew.cpp
  main.cpp
)

ADD_EXECUTABLE(uFldSearchDetectX ${SRC})

TARGET_LINK_LIBRARIES(uFldSearchDetectX
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

