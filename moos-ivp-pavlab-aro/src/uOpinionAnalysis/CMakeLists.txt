#--------------------------------------------------------
# The CMakeLists.txt for:                       uOpinionAnalysis
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  OpinionAnalysis.cpp
  OpinionAnalysis_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uOpinionAnalysis ${SRC})

TARGET_LINK_LIBRARIES(uOpinionAnalysis
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

