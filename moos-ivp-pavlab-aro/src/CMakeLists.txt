
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
# Let users control whether or not GUI-related libraries/apps are built.
#========================================================================
SET(BUILD_BOT_CODE_ONLY OFF CACHE BOOL "Build minimal set of bot apps.")

#========================================================================
# Let users control whether or Sandbox Apps/Libraries are built
#========================================================================
SET(BUILD_SANDBOX_CODE OFF CACHE BOOL "Build sandbox apps.")

#========================================================================
# Identify if cross repository dependencies exist
#========================================================================

find_library(HELMTASK_LIBRARY NAMES helmtask PATHS ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/lib NO_DEFAULT_PATH)
if(NOT HELMTASK_LIBRARY)
  message(STATUS "helmtask library not found")
  set(SWARM_TOOLBOX_AVAIL FALSE)
else()
  message(STATUS "helmtask library found")
  set(SWARM_TOOLBOX_AVAIL TRUE)
endif()


#============================================================================
# List the subdirectories to build...
#============================================================================

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/KayakEvalEngineUtils )
  ADD_SUBDIRECTORY(KayakEvalEngineUtils)
ENDIF()

#===============================================================
#===============================================================
# Set the ROBOT Apps
#===============================================================
SET(ROBOT_APPS lib_sock_utils)

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_eval_engine )
  LIST(APPEND ROBOT_APPS lib_eval_engine)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_survey )
  LIST(APPEND ROBOT_APPS lib_survey)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_serialdev )
  LIST(APPEND ROBOT_APPS lib_serialdev)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_seatrac_tools )
  LIST(APPEND ROBOT_APPS lib_seatrac_tools)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_convoy )
	LIST(APPEND ROBOT_APPS lib_bhv_convoy)
ENDIF()

# IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_anderson )
#   LIST(APPEND ROBOT_APPS lib_bhv_anderson)
# ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_pmview )
  LIST(APPEND ROBOT_APPS lib_bhv_pmview)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_navcheck )
  LIST(APPEND ROBOT_APPS lib_bhv_navcheck)
ENDIF()

#IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_legrunz )
#  LIST(APPEND ROBOT_APPS lib_bhv_legrunz)
#ENDIF()

#IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib_legrun)
#  LIST(APPEND ROBOT_APPS lib_legrun)
#ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_convoy )
  LIST(APPEND ROBOT_APPS lib_convoy)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/iM300 )
  LIST(APPEND ROBOT_APPS iM300)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/iBlueBoat )
  LIST(APPEND ROBOT_APPS iBlueBoat)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/iM300X )
  LIST(APPEND ROBOT_APPS iM300X)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/iVehX )
  LIST(APPEND ROBOT_APPS iVehX)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/app_pickjoust )
  LIST(APPEND ROBOT_APPS app_pickjoust)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/app_pluck )
  LIST(APPEND ROBOT_APPS app_pluck)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pTestPost )
  LIST(APPEND ROBOT_APPS pTestPost)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/iM300Health )
  LIST(APPEND ROBOT_APPS iM300Health)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pEvalConvoy )
  LIST(APPEND ROBOT_APPS pEvalConvoy)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pSafeRegion )
  LIST(APPEND ROBOT_APPS pSafeRegion)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pGeoSelect )
  LIST(APPEND ROBOT_APPS pGeoSelect)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pTrajectTranslate)
  LIST(APPEND ROBOT_APPS pTrajectTranslate)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pFalconRunMgr)
  LIST(APPEND ROBOT_APPS pFalconRunMgr)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_shell)
  LIST(APPEND ROBOT_APPS lib_bhv_shell)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/iHydroLinkArduinoBridge)
  LIST(APPEND ROBOT_APPS iHydroLinkArduinoBridge)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/iSeaTracX150V0)
  LIST(APPEND ROBOT_APPS iSeaTracX150V0)
ENDIF()

#IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uFldSearchDetectX )
#  LIST(APPEND ROBOT_APPS uFldSearchDetectX)
#ENDIF()


# IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_SearchBHV)
#   LIST(APPEND ROBOT_APPS lib_SearchBHV)
# ENDIF()

# IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_VectorBHV)
#   LIST(APPEND ROBOT_APPS lib_VectorBHV)
# ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_opinion)
  LIST(APPEND ROBOT_APPS lib_opinion)
ENDIF()

# Filip: did not build on pablos
# IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pOpinionManager)
#   LIST(APPEND ROBOT_APPS pOpinionManager)
# ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pOpinionInput)
  LIST(APPEND ROBOT_APPS pOpinionInput)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pOpinionFixer)
  LIST(APPEND ROBOT_APPS pOpinionFixer)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/iSerial)
  LIST(APPEND ROBOT_APPS iSerial)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uFldBloomStormSim)
  LIST(APPEND ROBOT_APPS uFldBloomStormSim)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_MoveToRegion)
  LIST(APPEND ROBOT_APPS lib_MoveToRegion)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pBloomStormCtrl)
  LIST(APPEND ROBOT_APPS pBloomStormCtrl)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/app_test_armadillo)
  LIST(APPEND ROBOT_APPS app_test_armadillo)
ENDIF()

#IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uFldJoustMgr)
#  LIST(APPEND ROBOT_APPS uFldJoustMgr)
#ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_dubin)
  LIST(APPEND ROBOT_APPS lib_dubin)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_demuster)
  LIST(APPEND ROBOT_APPS lib_bhv_demuster)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_dubin)
  LIST(APPEND ROBOT_APPS lib_bhv_dubin)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pDynamicTrafficLight)
  LIST(APPEND ROBOT_APPS pDynamicTrafficLight)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_demuster_geometry)
  LIST(APPEND ROBOT_APPS lib_demuster_geometry)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pDemusterAssign)
  LIST(APPEND ROBOT_APPS pDemusterAssign)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_hungarian_ext)
  LIST(APPEND ROBOT_APPS lib_hungarian_ext)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_random_survey)
  LIST(APPEND ROBOT_APPS lib_bhv_random_survey)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pAqctNodeCtrl)
  LIST(APPEND ROBOT_APPS pAqctNodeCtrl)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/dep_pMultiAgentColFilt)
  LIST(APPEND ROBOT_APPS dep_pMultiAgentColFilt)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pGroupComboAlloc)
  LIST(APPEND ROBOT_APPS pGroupComboAlloc)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uPopEval)
  LIST(APPEND ROBOT_APPS uPopEval)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uAdjPath)
  LIST(APPEND ROBOT_APPS uAdjPath)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_path_plan)
  LIST(APPEND ROBOT_APPS lib_path_plan)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uOptionMarker)
  LIST(APPEND ROBOT_APPS uOptionMarker)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uUtilVectorMarker)
  LIST(APPEND ROBOT_APPS uUtilVectorMarker)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uFldEvalOption)
  LIST(APPEND ROBOT_APPS uFldEvalOption)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pSumVars)
  LIST(APPEND ROBOT_APPS pSumVars)
ENDIF()

#IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_random_survey2)
#  LIST(APPEND ROBOT_APPS lib_bhv_random_survey2)
#ENDIF()

#IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_votetask)
#  LIST(APPEND ROBOT_APPS lib_votetask)
#ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pVoteManager)
  LIST(APPEND ROBOT_APPS pVoteManager)
ENDIF()

# NB: NOT BUILDING ON UBUNTU
IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pTurnInPlace)
  LIST(APPEND ROBOT_APPS pTurnInPlace)
ENDIF()

#===============================================================
# Set the SANDBOX Apps
#===============================================================
SET(SANDBOX_APPS )

IF( EXISTS ${IVP_EXTEND_PARENT_DIR}/HydroMAN)
  LIST(APPEND SANDBOX_APPS iHydroMAN_IvPExtend)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uChangeMyName)
  LIST(APPEND SANDBOX_APPS uChangeMyName)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_learning)
  LIST(APPEND SANDBOX_APPS lib_learning)
ENDIF()


IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pDynamLearning)
  LIST(APPEND SANDBOX_APPS pDynamLearning)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/iPingSim)
  LIST(APPEND SANDBOX_APPS iPingSim)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bathygrid)
  LIST(APPEND SANDBOX_APPS lib_bathygrid)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uFldSearchDetect)
  LIST(APPEND SANDBOX_APPS uFldSearchDetect)
ENDIF()

#IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pBathyGrider)
#  LIST(APPEND SANDBOX_APPS pBathyGrider)
#ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pBathyPath)
  LIST(APPEND SANDBOX_APPS pBathyPath)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pGridSwitcher)
  LIST(APPEND SANDBOX_APPS pGridSwitcher)
ENDIF()

#IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pRoutePlan)
#  LIST(APPEND SANDBOX_APPS pRoutePlan)
#ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pBathyChecker)
  LIST(APPEND SANDBOX_APPS pBathyChecker)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/iBlueRoboticsPing)
  IF( EXISTS /home/student2680/ping-cpp)
    LIST(APPEND SANDBOX_APPS iBlueRoboticsPing)
  ENDIF()
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pKalmanSolutionGen)
  LIST(APPEND SANDBOX_APPS pKalmanSolutionGen)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pSonarSimDetect)
  LIST(APPEND SANDBOX_APPS pSonarSimDetect)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pThrusterAlloc)
  LIST(APPEND SANDBOX_APPS pThrusterAlloc)
ENDIF()

#IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uDisplayLLRegion)
#  LIST(APPEND SANDBOX_APPS uDisplayLLRegion)
#ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uOpinionAnalysis)
  LIST(APPEND SANDBOX_APPS uOpinionAnalysis)
ENDIF()


#===============================================================
# Set the OTHER Apps
#===============================================================

SET(OTHER_APPS uSimHeron)

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uSockComms )
  LIST(APPEND OTHER_APPS uSockComms)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/app_alogceval )
  LIST(APPEND OTHER_APPS app_alogceval)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/app_convoy_order )
  LIST(APPEND OTHER_APPS app_convoy_order)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/uFldConvoyEval )
  LIST(APPEND OTHER_APPS uFldConvoyEval)
ENDIF()

SET(APPS_TO_BUILD ${ROBOT_APPS})

#===============================================================
# SWARM TOOLBOX DEPENDENCIES
#===============================================================

SET(SWARM_DEPENDS)

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_convoypd AND SWARM_TOOLBOX_AVAIL)
  LIST(APPEND SWARM_DEPENDS lib_bhv_convoypd)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_tasks_d2p AND SWARM_TOOLBOX_AVAIL)
  LIST(APPEND SWARM_DEPENDS lib_bhv_tasks_d2p)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_tasks_comp AND SWARM_TOOLBOX_AVAIL)
  LIST(APPEND SWARM_DEPENDS lib_bhv_tasks_comp)
ENDIF()

IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_bhv_tasks_dubins AND SWARM_TOOLBOX_AVAIL)
  LIST(APPEND SWARM_DEPENDS lib_bhv_tasks_dubins)
ENDIF()


IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pHVUProtectAssetCtrl AND SWARM_TOOLBOX_AVAIL)
  LIST(APPEND SWARM_DEPENDS pHVUProtectAssetCtrl)
ENDIF()

# Filip: did not build on pablos
# IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pHVUProtectNodeCtrl AND SWARM_TOOLBOX_AVAIL)
#   LIST(APPEND SWARM_DEPENDS pHVUProtectNodeCtrl)
# ENDIF()

# Filip: did not build on pablos
# IF( EXISTS ${CMAKE_SOURCE_DIR}/src/pProxonoiGridSearch)
#   LIST(APPEND SWARM_DEPENDS pProxonoiGridSearch)
# ENDIF()


message("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
message(" SRC: BUILD_BOT_CODE_ONLY: ${BUILD_BOT_CODE_ONLY}")
message(" SRC: BUILD_SANDBOX_CODE:  ${BUILD_SANDBOX_CODE}")
message("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

IF("${BUILD_BOT_CODE_ONLY}" STREQUAL "OFF")
  LIST(APPEND APPS_TO_BUILD ${OTHER_APPS})
ENDIF("${BUILD_BOT_CODE_ONLY}" STREQUAL "OFF")

IF("${BUILD_SANDBOX_CODE}" STREQUAL "ON")
  LIST(APPEND APPS_TO_BUILD ${SANDBOX_APPS})
ENDIF("${BUILD_SANDBOX_CODE}" STREQUAL "ON")

IF(${SWARM_TOOLBOX_AVAIL})
  LIST(APPEND APPS_TO_BUILD ${SWARM_DEPENDS})
ENDIF(${SWARM_TOOLBOX_AVAIL})


FOREACH(A ${APPS_TO_BUILD})
  message("APP:" ${A} )
  ADD_SUBDIRECTORY(${A})
ENDFOREACH(A)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
