#--------------------------------------------------------
# The CMakeLists.txt for:                       pTrajectTranslate
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  TrajectTranslate.cpp
  TrajectTranslate_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pTrajectTranslate ${SRC})

TARGET_LINK_LIBRARIES(pTrajectTranslate
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

