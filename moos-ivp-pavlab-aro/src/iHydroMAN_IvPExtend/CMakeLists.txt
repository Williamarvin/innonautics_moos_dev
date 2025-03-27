#--------------------------------------------------------
# The CMakeLists.txt for:                       iHydroMAN_IvPExtend
# Author(s):                              Supun Randeni
#--------------------------------------------------------

# enable C++11 (see top-level CMakeLists.txt for macro definition)
use_cxx11()

SET(SRC
  HydroMAN_IvPExtend.cpp
  HydroMAN_IvPExtend_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iHydroMAN_IvPExtend ${SRC})

TARGET_LINK_LIBRARIES(iHydroMAN_IvPExtend
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   ${Boost_LIBRARIES}
   contacts
   geometry
   apputil
   mbutil
   m
   pthread
   hydroman_tcp
   hydroman_protobuf
   protobuf
   )

