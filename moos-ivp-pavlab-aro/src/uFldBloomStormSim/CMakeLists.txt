#--------------------------------------------------------
# The CMakeLists.txt for:                       uFldBloomStormSim
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  FldBloomStormSim.cpp
  FldBloomStormSim_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uFldBloomStormSim ${SRC})

TARGET_LINK_LIBRARIES(uFldBloomStormSim
  contacts
  geometry
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

