#--------------------------------------------------------
# The CMakeLists.txt for:                       pFalconRunMgr
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  FalconRunMgr.cpp
  FalconRunMgr_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pFalconRunMgr ${SRC})

TARGET_LINK_LIBRARIES(pFalconRunMgr
  ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

