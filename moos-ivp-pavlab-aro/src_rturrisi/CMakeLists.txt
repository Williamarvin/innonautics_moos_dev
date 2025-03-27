
##############################################################################
# FILE:        moos-ivp-pavlab/src/CMakeLists.txt
# DATE:        2020/03/22
# DESCRIPTION: CMakeLists.txt file for the moos-ivp-pavlab source directory
##############################################################################


#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})

#========================================================================
# Identify if cross repository dependencies exist
#========================================================================

find_library(HELMTASK_LIBRARY NAMES helmtask PATHS ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/lib NO_DEFAULT_PATH)

if(NOT HELMTASK_LIBRARY)
  message(STATUS "\"helmtask\" library not found - will not build swarm toolbox dependencies")
  set(SWARM_TOOLBOX_AVAIL FALSE)
else()
  set(SWARM_TOOLBOX_AVAIL TRUE)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#===============================================================
#===============================================================
# Set the ROBOT Apps
#===============================================================
IF( EXISTS ${CMAKE_SOURCE_DIR}/src_rturrisi/uDashFast )
  ADD_SUBDIRECTORY(uDashFast)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src_rturrisi/lib_bhv_formations )
  ADD_SUBDIRECTORY(lib_bhv_formations)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src_rturrisi/lib_bhv_fig8 )
  ADD_SUBDIRECTORY(lib_bhv_fig8)
ENDIF()

#===============================================================
# SWARM TOOLBOX DEPENDENCIES
#===============================================================

SET(SWARM_DEPENDS)

IF( EXISTS ${CMAKE_SOURCE_DIR}/src_rturrisi/lib_bhv_task_formations AND SWARM_TOOLBOX_AVAIL)
  ADD_SUBDIRECTORY(lib_bhv_task_formations)
ENDIF()



SET(APPS_TO_BUILD ${ROBOT_APPS})


message("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
message(" SRC: BUILD_BOT_CODE_ONLY: ${BUILD_BOT_CODE_ONLY}")
message(" SRC: BUILD_GUI_CODE:  ${BUILD_GUI_CODE}")
message("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

IF("${BUILD_BOT_CODE_ONLY}" STREQUAL "OFF")
  LIST(APPEND APPS_TO_BUILD ${GUI_APPS})
ENDIF("${BUILD_BOT_CODE_ONLY}" STREQUAL "OFF")

FOREACH(A ${APPS_TO_BUILD})
  message("APP:" ${A} )
  ADD_SUBDIRECTORY(${A})
ENDFOREACH(A)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
