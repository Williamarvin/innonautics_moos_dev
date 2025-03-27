#--------------------------------------------------------
# The CMakeLists.txt for:                       uFldEvalOption
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  FldEvalOption.cpp
  FldEvalOption_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uFldEvalOption ${SRC})

TARGET_LINK_LIBRARIES(uFldEvalOption
  ${MOOS_LIBRARIES}
  contacts
  geometry
   apputil
   mbutil
   m
   pthread)

