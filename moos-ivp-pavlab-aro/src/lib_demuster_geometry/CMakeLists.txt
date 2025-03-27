# -----------------------------------------------------------------------------
# The CMakeLists.txt for       lib_demuster_geometry Author(s): Filip Stromstad
# -----------------------------------------------------------------------------

# enable C++11 (see top-level CMakeLists.txt for macro definition)
use_cxx11()

set(SRC demuster_geometry.cpp)

set(HEADERS demuster_geometry.h)

# Build Library
add_library(demuster_geometry ${SRC})