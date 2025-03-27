#--------------------------------------------------------
# The CMakeLists.txt for:                 lib_path_plan
# Author(s):                              Tyler Paine
#--------------------------------------------------------



SET(SRC
  SimpleAStar.cpp
  SimpleMDP.cpp
  GraphNode.cpp
  )

SET(HEADERS
  SimpleAStar.h
  SimpleMDP.h
  GraphNode.h
)

# Build Library
ADD_LIBRARY(path_plan ${SRC})
