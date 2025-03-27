#--------------------------------------------------------
# The CMakeLists.txt for:                       pSumVars
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  SumVars.cpp
  SumVars_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSumVars ${SRC})

TARGET_LINK_LIBRARIES(pSumVars
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

