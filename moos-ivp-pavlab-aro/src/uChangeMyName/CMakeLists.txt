#--------------------------------------------------------
# The CMakeLists.txt for:                       uChangeMyName
# Author(s):                              Supun Randeni
#--------------------------------------------------------

SET(SRC
  ChangeMyName.cpp
  ChangeMyName_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uChangeMyName ${SRC})

TARGET_LINK_LIBRARIES(uChangeMyName
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

