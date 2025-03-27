#--------------------------------------------------------
# The CMakeLists.txt for:                       pSonarSimDetect
# Author(s):                              Craig Evans
#--------------------------------------------------------

SET(SRC
  SonarSimDetect.cpp
  SonarSimDetect_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSonarSimDetect ${SRC})

TARGET_LINK_LIBRARIES(pSonarSimDetect
   ${MOOS_LIBRARIES}
   apputil
   contacts
   geometry
   mbutil
   m
   pthread)

