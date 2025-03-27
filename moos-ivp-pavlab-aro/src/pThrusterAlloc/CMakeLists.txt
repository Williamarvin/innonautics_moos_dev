#--------------------------------------------------------
# The CMakeLists.txt for:                       pThrusterAlloc
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  ThrusterAlloc.cpp
  ThrusterAlloc_Info.cpp
  Thruster.cpp
  main.cpp
)

ADD_EXECUTABLE(pThrusterAlloc ${SRC})

TARGET_LINK_LIBRARIES(pThrusterAlloc
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

