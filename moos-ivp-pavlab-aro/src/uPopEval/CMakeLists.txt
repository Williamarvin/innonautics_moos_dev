#--------------------------------------------------------
# The CMakeLists.txt for:                       uPopEval
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  PopEval.cpp
  PopEval_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uPopEval ${SRC})

TARGET_LINK_LIBRARIES(uPopEval
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

