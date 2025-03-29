#--------------------------------------------------------
# The CMakeLists.txt for: pMarineViewer
# Author(s): Mike Benjamin
#--------------------------------------------------------

cmake_minimum_required(VERSION 3.10)
project(pMarineViewer)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set System-Specific Libraries
if (WIN32)
    set(SYSTEM_LIBS wsock32)
else()
    set(SYSTEM_LIBS m)
endif()

if (CMAKE_SYSTEM_NAME STREQUAL Linux)
    set(SYSTEM_LIBS ${SYSTEM_LIBS} GL)
endif()

# Locate All Source Files
file(GLOB SRC *.cpp)

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find FLTK
find_package(FLTK REQUIRED)
include_directories(${FLTK_INCLUDE_DIRS})

# Find libcurl
find_package(CURL REQUIRED)
include_directories(${CURL_INCLUDE_DIRS})

# Define Executable
add_executable(pMarineViewer ${SRC})

# Link Required Libraries
target_link_libraries(pMarineViewer
    ${MOOS_LIBRARIES}
    ${MOOSGeodesy_LIBRARIES}
    ${SYSTEM_LIBS}
    genutil
    marineview
    geodaid
    contacts
    geometry
    ucommand
    realm
    apputil
    mbutil
    bhvutil
    ${OpenCV_LIBS}
    ${FLTK_LIBRARIES}
    ${CURL_LIBRARIES}   # Link libcurl
    fltk
    fltk_gl
    dl
    tiff
)

# Ensure Correct Linking for macOS
if (APPLE)
    set_target_properties(pMarineViewer PROPERTIES
        LINK_FLAGS "-framework OpenGL -framework AGL -framework Cocoa"
    )
endif()

# Run a Test After Compilation
add_test(MyTest1 ${CMAKE_BINARY_DIR}/pMarineViewer --version)