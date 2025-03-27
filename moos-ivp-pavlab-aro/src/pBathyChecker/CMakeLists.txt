#--------------------------------------------------------
# The CMakeLists.txt for:                       pBathyChecker
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  BathyChecker.cpp
  BathyChecker_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pBathyChecker ${SRC})

TARGET_LINK_LIBRARIES(pBathyChecker
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

