#--------------------------------------------------------
# The CMakeLists.txt for:                       uUtilVectorMarker
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  UtilVectorMarker.cpp
  UtilVectorMarker_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uUtilVectorMarker ${SRC})

TARGET_LINK_LIBRARIES(uUtilVectorMarker
  ${MOOS_LIBRARIES}
  geometry
   apputil
   mbutil
   m
   pthread)

