
##############################################################################
# FILE:        moos-ivp-pavlab/src/KayakEvalEngineUtils/CMakeLists.txt
# DATE:        2023/08/20
# DESCRIPTION: CMakeLists.txt file for the realtime and post processing utility
#              EvalEngine template files
##############################################################################


#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})

#============================================================================
# List the subdirectories to build...
#============================================================================

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/KayakEvalEngineUtils/app_postKayakEvalEngine )
  LIST(APPEND ROBOT_APPS app_postKayakEvalEngine)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/KayakEvalEngineUtils/pKayakEvalEngine )
  LIST(APPEND ROBOT_APPS pKayakEvalEngine)
ENDIF()

SET(APPS_TO_BUILD ${ROBOT_APPS})

IF("${BUILD_BOT_CODE_ONLY}" STREQUAL "OFF")
  LIST(APPEND APPS_TO_BUILD ${OTHER_APPS})
ENDIF("${BUILD_BOT_CODE_ONLY}" STREQUAL "OFF")

IF("${BUILD_SANDBOX_CODE}" STREQUAL "ON")
  LIST(APPEND APPS_TO_BUILD ${SANDBOX_APPS})
ENDIF("${BUILD_SANDBOX_CODE}" STREQUAL "ON")


FOREACH(A ${APPS_TO_BUILD})
  message("APP:" ${A} )
  ADD_SUBDIRECTORY(${A})
ENDFOREACH(A)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
