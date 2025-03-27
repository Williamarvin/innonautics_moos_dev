#--------------------------------------------------------
# The CMakeLists.txt for:                       pKayakEvalEngine
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  KayakEvalEngine.cpp
  main.cpp
)

ADD_EXECUTABLE(pKayakEvalEngine ${SRC})

TARGET_LINK_LIBRARIES(pKayakEvalEngine
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread
   eval_engine)

