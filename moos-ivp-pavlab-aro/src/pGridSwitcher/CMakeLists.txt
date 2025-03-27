#--------------------------------------------------------
# The CMakeLists.txt for:                       pGridSwitcher
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  GridSwitcher.cpp
  GridSwitcher_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pGridSwitcher ${SRC})

TARGET_LINK_LIBRARIES(pGridSwitcher
  ${MOOS_LIBRARIES}
  MOOSGeodesy
  geometry
  apputil
  mbutil
  m
  pthread
  bathygrid)

