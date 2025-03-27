# --------------------------------------------------------------------------------
# The CMakeLists.txt for:             lib_hungarian_ext Author(s): Filip Stromstad
# --------------------------------------------------------------------------------

# enable C++11 (see top-level CMakeLists.txt for macro definition)
use_cxx11()

set(SRC Hungarian.cpp)

set(HEADERS Hungarian.h)

# Build Library
add_library(hungarian_ext ${SRC})