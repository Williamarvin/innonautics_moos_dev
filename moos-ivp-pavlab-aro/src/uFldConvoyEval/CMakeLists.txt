#--------------------------------------------------------
# The CMakeLists.txt for:                  uFldConvoyEval
# Author(s):                                  Tyler Paine
#--------------------------------------------------------

SET(SRC
  ConvoyEval_MOOSApp.cpp
  ConvoyEval_Info.cpp
  main.cpp
  FldEvalConvoyEngine.cpp
)

ADD_EXECUTABLE(uFldConvoyEval ${SRC})

TARGET_LINK_LIBRARIES(uFldConvoyEval
   ${MOOS_LIBRARIES}
   convoy
   contacts
   geometry
   ufield
   apputil
   mbutil
   m
   pthread)

