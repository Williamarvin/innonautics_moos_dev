#--------------------------------------------------------
# The CMakeLists.txt for:                     pEvalConvoy
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  EvalConvoy.cpp
  EvalConvoy_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pEvalConvoy ${SRC})

TARGET_LINK_LIBRARIES(pEvalConvoy
   ${MOOS_LIBRARIES}
   convoy
   contacts
   geometry
   ufield
   apputil
   mbutil
   m
   pthread)

