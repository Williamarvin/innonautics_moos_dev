#--------------------------------------------------------
# The CMakeLists.txt for:                   lib_bathygrid
# Author(s):                  Tyler Paine, Nick Gershfeld
#--------------------------------------------------------

SET(SRC
  EsriBathyGrid.cpp
  EsriBathyGridUtils.cpp)

ADD_LIBRARY(bathygrid ${SRC} )

TARGET_LINK_LIBRARIES(bathygrid
  geometry
  path_plan)

