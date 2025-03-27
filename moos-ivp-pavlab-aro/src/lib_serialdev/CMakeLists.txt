# --------------------------------------------------------
# The CMakeLists.txt for:                   lib_serialdev Author(s): Ray Turrisi
# --------------------------------------------------------

set(SRC serial_device.cpp)

set(HEADERS serial_device.h)

# Build Library
add_library(serialdev ${SRC})
