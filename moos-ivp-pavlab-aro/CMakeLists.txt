#=======================================================================
# FILE:  moos-ivp-pavlab/CMakeLists.txt
# DATE:  2012/07/24
# INFO:  Top-level CMakeLists.txt file for the moos-ivp-pavlab project
#=======================================================================

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT( IVP_EXTEND )

# NOTE! The following macro permits users to enable C++11 selectively,
#       only where necessary, for specific MOOSApps.  To enable C++11,
#       add the following line to the application's CMakeLists.txt file:
#       use_cxx11()

macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)

# Check if Armadillo and dependencies are available - set global environment variables to streamline build files
find_package(BLAS)
find_package(LAPACK)

if(BLAS_FOUND AND LAPACK_FOUND)
    set(ARMA_AVAIL TRUE)
    SET(ARMADILLO_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/src/lib_armadillo/armadillo-12.6.6/include")
    add_definitions(-DARMA_DONT_USE_WRAPPER)
    SET(ARMA_LIBS ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
else()
   set(ARMA_AVAIL FALSE)
   message(WARNING "BLAS and LAPACK library not found - No applications which depend on Armadillo will be built. \n Try: \n\t - Mac: <brew install openblas> \n\t - Ubuntu/Debian: <sudo apt install libblas-dev liblapack-dev>")
endif()

# adapted from:
# <https://stackoverflow.com/questions/10851247/how-do-i-activate-c-11-in-cmake>

# added by: Blake Cole [29 APRIL 2021]

#=======================================================================
# Set the output directories for the binary and library files
#=======================================================================

GET_FILENAME_COMPONENT(IVP_EXTEND_BIN_DIR "${CMAKE_SOURCE_DIR}/bin"  ABSOLUTE )
GET_FILENAME_COMPONENT(IVP_EXTEND_LIB_DIR "${CMAKE_SOURCE_DIR}/lib"  ABSOLUTE )
GET_FILENAME_COMPONENT(IVP_EXTEND_PARENT_DIR .. ABSOLUTE)

SET( LIBRARY_OUTPUT_PATH      "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )
SET( ARCHIVE_OUTPUT_DIRECTORY "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )
SET( LIBRARY_OUTPUT_DIRECTORY "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )

SET( EXECUTABLE_OUTPUT_PATH    "${IVP_EXTEND_BIN_DIR}" CACHE PATH "" )
SET( RUNTIME_OUTPUT_DIRECTORY "${IVP_EXTEND_BIN_DIR}"  CACHE PATH "" )

#=======================================================================
# Find MOOS
#=======================================================================
find_package(MOOS 10.0)
INCLUDE_DIRECTORIES(${MOOS_INCLUDE_DIRS})

#=============================================================================      
# FINDING MOOSGeodesy' HEADERS AND LIBRARIES...                                     
#=============================================================================      
find_package(MOOSGeodesy)
include_directories(${MOOSGeodesy_INCLUDE_DIRS})
link_directories(${MOOSGeodesy_LIBRARY_PATH})

message("+++++++++++++++++++++++++++++++++++++++++")
message("MOOSGeodesy_INCLUDE_DIRS:" ${MOOSGeodesy_INCLUDE_DIRS})
message("MOOSGeodesy_LIBRARY_PATH:" ${MOOSGeodesy_LIBRARY_PATH})
message("+++++++++++++++++++++++++++++++++++++++++")

#=======================================================================
# Find the "moos-ivp" base directory
#=======================================================================

# Search for the moos-ivp folder
find_path( MOOSIVP_SOURCE_TREE_BASE
           NAMES build-ivp.sh build-moos.sh configure-ivp.sh
           PATHS "../moos-ivp" "../../moos-ivp" "../../moos-ivp/trunk/" "../moos-ivp/trunk/"
           DOC "Base directory of the MOOS-IvP source tree"
           NO_DEFAULT_PATH
)

if (NOT MOOSIVP_SOURCE_TREE_BASE)
    message("Please set MOOSIVP_SOURCE_TREE_BASE to  ")
    message("the location of the \"moos-ivp\" folder ")
    return()
endif()

#======================================================================
# Specify where to find IvP's headers and libraries...
#======================================================================

FILE(GLOB IVP_INCLUDE_DIRS ${MOOSIVP_SOURCE_TREE_BASE}/ivp/src/lib_* )
INCLUDE_DIRECTORIES(${IVP_INCLUDE_DIRS})

FILE(GLOB IVP_LIBRARY_DIRS ${MOOSIVP_SOURCE_TREE_BASE}/lib )
LINK_DIRECTORIES(${IVP_LIBRARY_DIRS})


#=============================================================================
# Tell CMake (and thus C++) where to find IvP's header files...
#=============================================================================

IF( EXISTS /usr/local/include AND EXISTS /usr/local/lib )
  INCLUDE_DIRECTORIES(/usr/local/include)
  LINK_DIRECTORIES(/usr/local/lib)
ENDIF()

IF( EXISTS /opt/local/include AND EXISTS /opt/local/lib )
  INCLUDE_DIRECTORIES(/opt/local/include)
  LINK_DIRECTORIES(/opt/local/lib)
ENDIF()

IF( EXISTS /opt/homebrew/include AND EXISTS /opt/homebrew/lib )
  INCLUDE_DIRECTORIES(/opt/homebrew/include)
  LINK_DIRECTORIES(/opt/homebrew/lib)
ENDIF()




#======================================================================
# Specify where to find HydroMAN's headers and libraries...
# Link protocol buffers as a package
#======================================================================
IF( EXISTS ${IVP_EXTEND_PARENT_DIR}/HydroMAN)
   # Where to find cmake module path
   set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/")

   # Link protocol buffers first - http://code.google.com/apis/protocolbuffers/
   find_package(ProtobufLocal REQUIRED)
   include_directories(${PROTOBUF_INCLUDE_DIRS})

   if(${PROTOC_VERSION} VERSION_LESS 2.5.0})
     message("libprotobuf < 2.5.0")
   else()
     message("libprotobuf >= 2.5.0")
     set(PROTOBUF_ALLOW_ALIAS "option allow_alias = true;")
   endif()

   # boost - http://www.boost.org/
   find_package(Boost 1.55.0 REQUIRED date_time thread system program_options filesystem regex serialization)

   # Specify where to find HydroMAN's headers and libraries...
   get_filename_component(HYDROMAN_INC_DIR  ${IVP_EXTEND_PARENT_DIR}/HydroMAN/include ABSOLUTE)
   get_filename_component(HYDROMAN_LIB_DIR  ${IVP_EXTEND_PARENT_DIR}/HydroMAN/lib ABSOLUTE)
   include_directories(${HYDROMAN_INC_DIR})
   LINK_DIRECTORIES(${HYDROMAN_LIB_DIR})
   protobuf_include_dirs(${HYDROMAN_INC_DIR})
   message ("Found HydroMAN 2.0 in ${HYDROMAN_INC_DIR}")  
ENDIF()

#======================================================================
# Specify Compiler Flags
#======================================================================
IF( ${WIN32} )
   #---------------------------------------------
   # Windows Compiler Flags
   #---------------------------------------------
   IF(MSVC)
      # Flags for Microsoft Visual Studio
      SET( WALL_ON OFF CACHE BOOL 
         "tell me about all compiler warnings (-Wall) ")
      IF(WALL_ON)
         SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
      ENDIF(WALL_ON)
   ELSE(MSVC)
      # Other Windows compilers go here
   ENDIF(MSVC)

ELSE( ${WIN32} )
   #---------------------------------------------
   # Linux and Apple Compiler Flags
   #---------------------------------------------
   # Force -fPIC because gcc complains when we don't use it with x86_64 code.
   # Note sure why: -fPIC should only be needed for shared objects, and
   # AFAIK, CMake gets that right when building shared objects. -CJC
   # -Wno-psabi turns off warnings about ABI change between gcc 6 and 7.1
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -g -Wno-psabi ")
   IF(CMAKE_COMPILER_IS_GNUCXX)
      # Flags for the GNU C++ Compiler
      SET( WALL_ON OFF CACHE BOOL 
         "tell me about all compiler warnings (-Wall) ")
      IF(WALL_ON)
         SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall" -C++11)
      ENDIF( WALL_ON)
   ELSE(CMAKE_COMPILER_IS_GNUCXX)
    
   ENDIF(CMAKE_COMPILER_IS_GNUCXX)

ENDIF( ${WIN32} )

#=======================================================================
# Add Subdirectories
#=======================================================================
ADD_SUBDIRECTORY( src )
IF( EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src_private )
  ADD_SUBDIRECTORY( src_private )
ENDIF()
 IF( EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src_sailing )
   ADD_SUBDIRECTORY( src_sailing )
 ENDIF()
IF( EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src_rturrisi )
  ADD_SUBDIRECTORY( src_rturrisi )
ENDIF()

# IF( EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src_mikala )
#   ADD_SUBDIRECTORY( src_mikala )
# ENDIF()

#IF( EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src_joystick )
#  ADD_SUBDIRECTORY( src_joystick )
#ENDIF()
