#--------------------------------------------------------
# The CMakeLists.txt for:                       iHydroLink
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  HydroLinkArduinoBridge.cpp
  HydroLinkArduinoBridge_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iHydroLinkArduinoBridge ${SRC})

TARGET_LINK_LIBRARIES(iHydroLinkArduinoBridge
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   contacts
   geometry
   mbutil
   m
   pthread)

