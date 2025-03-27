# -----------------------------------------------------------------------
# The CMakeLists.txt for:            lib_dubin Author(s): Filip Stromstad
# -----------------------------------------------------------------------

# enable C++11 (see top-level CMakeLists.txt for macro definition)
use_cxx11()

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})

set(SRC dubin.cpp)

set(HEADERS dubin.h)



# Build Library
add_library(dubin ${SRC})