#--------------------------------------------------------
# The CMakeLists.txt for:                      lib_legrun
# Author(s):                                Mike Benjamin 
#--------------------------------------------------------

SET(SRC
  LegRun.cpp
  LegRunSet.cpp
)

# Build Library
ADD_LIBRARY(legrun ${SRC})
