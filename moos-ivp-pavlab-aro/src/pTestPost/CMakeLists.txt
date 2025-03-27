#--------------------------------------------------------
# The CMakeLists.txt for:                       pTestPost
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  TestPost.cpp
  TestPost_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pTestPost ${SRC})

TARGET_LINK_LIBRARIES(pTestPost
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

