#--------------------------------------------------------
# The CMakeLists.txt for:                       pKalmanSolutionGen
# Author(s):                              Craig Evans
#--------------------------------------------------------

SET(SRC
  KalmanSolutionGen.cpp
  KalmanSolutionGen_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pKalmanSolutionGen ${SRC})

TARGET_LINK_LIBRARIES(pKalmanSolutionGen
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

