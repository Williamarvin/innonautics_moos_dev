#--------------------------------------------------------
# The CMakeLists.txt for:                pGroupComboAlloc
# Author(s):                              Tyler Paine
#--------------------------------------------------------

# enable C++11 (see top-level CMakeLists.txt for macro definition)
use_cxx11()

SET(SRC
  GroupComboAlloc.cpp
  GroupComboAlloc_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pGroupComboAlloc ${SRC})

TARGET_LINK_LIBRARIES(pGroupComboAlloc
  ${MOOS_LIBRARIES}
  hungarian_ext
  contacts
  geometry
  apputil
  mbutil
  m
  pthread)

