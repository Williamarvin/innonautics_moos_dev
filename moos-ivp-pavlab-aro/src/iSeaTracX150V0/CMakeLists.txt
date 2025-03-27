#--------------------------------------------------------
# The CMakeLists.txt for:                       iSeaTracX150V0
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  iSeaTracX150V0.cpp
  iSeaTracX150V0_Info.cpp
  main.cpp
)


ADD_EXECUTABLE(iSeaTracX150V0 ${SRC})

target_include_directories(iSeaTracX150V0 PUBLIC
  ${CMAKE_SOURCE_DIR}/src/lib_seatrac_tools/BpSystem/Linux
  ${CMAKE_SOURCE_DIR}/src/lib_seatrac_tools/BpClasses
  ${CMAKE_SOURCE_DIR}/src/lib_seatrac_tools/BpSeaTrac
)

TARGET_LINK_LIBRARIES(iSeaTracX150V0
   contacts
   geometry
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   mbutil
   seatrac_tools
   serialdev
   m
   pthread)

