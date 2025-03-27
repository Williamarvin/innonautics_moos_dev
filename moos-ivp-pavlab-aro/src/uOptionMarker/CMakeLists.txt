#--------------------------------------------------------
# The CMakeLists.txt for:                       uOptionMarker
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  OptionMarker.cpp
  OptionMarker_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uOptionMarker ${SRC})

TARGET_LINK_LIBRARIES(uOptionMarker
  ${MOOS_LIBRARIES}
  geometry
  opinion
   apputil
   mbutil
   m
   pthread)

