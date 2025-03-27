#--------------------------------------------------------
# The CMakeLists.txt for:                       pOpinionFixer
# Author(s):                              Tyler Paine
#--------------------------------------------------------

SET(SRC
  OpinionFixer.cpp
  OpinionFixer_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pOpinionFixer ${SRC})

TARGET_LINK_LIBRARIES(pOpinionFixer
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

