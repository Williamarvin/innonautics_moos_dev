# --------------------------------------------------------
# The CMakeLists.txt for:                   lib_serialdev Author(s): Ray Turrisi
# --------------------------------------------------------

set(SRC
    BpClasses/conversion.cpp
    BpClasses/crc16.cpp
    BpClasses/crc32.cpp
    BpClasses/maths.cpp
    BpClasses/maths.cpp2
    BpClasses/serialize.cpp
    BpClasses/serializebuffer.cpp
    BpSeatrac/seatrac_beacon.cpp
    BpSeatrac/seatrac_cmdmsg.cpp
    BpSeatrac/seatrac_cmdproc.cpp
    BpSeatrac/seatrac_diagmsg.cpp
    BpSeatrac/seatrac_utils.cpp
    BpSystem/Linux/mutex.cpp
    BpSystem/Linux/ticks.cpp)

set(HEADERS BpSeaTrac/seatrac_beacon.hpp BpSeaTrac/seatrac_types.h)

# Build Library
add_library(seatrac_tools ${SRC})

target_include_directories(
  seatrac_tools
  PRIVATE ${CMAKE_SOURCE_DIR}/src/lib_seatrac_tools/BpSystem/Linux
          ${CMAKE_SOURCE_DIR}/src/lib_seatrac_tools/BpClasses
          ${CMAKE_SOURCE_DIR}/src/lib_seatrac_tools/BpSeaTrac)

# This library has tons of warnings, so suppress all of them for our convenience
target_compile_options(seatrac_tools PRIVATE -w)
