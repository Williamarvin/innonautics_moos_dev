#--------------------------------------------------------
# The CMakeLists.txt for:                      pGeoSelect
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  GeoSelect.cpp
  GeoSelect_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pGeoSelect ${SRC})

TARGET_LINK_LIBRARIES(pGeoSelect
   ${MOOS_LIBRARIES}
   apputil
   geometry
   mbutil
   m
   pthread)

