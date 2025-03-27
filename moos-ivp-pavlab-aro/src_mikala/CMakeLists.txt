##############################################################################
# FILE:        moos-ivp-pavlab/src_mikala/CMakeLists.txt
# DATE:        2021/07/16
# DESCRIPTION: CMakeLists.txt file for moos-ivp-pavlab src_mikala directory
##############################################################################


#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS ./lib_* ../src/lib_*)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})

#============================================================================
# List the subdirectories to build...
#============================================================================
IF( EXISTS ${CMAKE_SOURCE_DIR}/src_mikala/pManagePoints )
  ADD_SUBDIRECTORY(pManagePoints)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src_mikala/uFldColregsDetect )
  ADD_SUBDIRECTORY(uFldColregsDetect)
ENDIF()

# IF( EXISTS ${CMAKE_SOURCE_DIR}/src_mikala/pManagePoints2 )
#   ADD_SUBDIRECTORY(pManagePoints2)
# ENDIF()

# IF( EXISTS ${CMAKE_SOURCE_DIR}/src_mikala/NodeReportUtils )
#   ADD_SUBDIRECTORY(NodeReportUtils)
# ENDIF()


#===============================================================
# Set the ROBOT Apps

##############################################################################
#                           END of CMakeLists.txt
##############################################################################

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

