# --------------------------------------------------------
# The CMakeLists.txt for:                 lib_eval_engine Author(s): Ray Turrisi
# --------------------------------------------------------

set(SRC
    EvalEngine.cpp
    EvalEngineKernelCore.cpp
    EvalEngineMessage.cpp
    EvalEnginePostProcessor.cpp
    std_ee_kernels/xCounter.cpp
    std_ee_kernels/xStateTimer.cpp
    std_ee_kernels/xSurfaceOdometry.cpp)

set(HEADERS
    EvalEngine.h
    EvalEngineKernelCore.h
    EvalEngineMessage.h
    std_ee_kernels.h
    std_ee_kernels/xCounter.h
    std_ee_kernels/xStateTimer.h
    std_ee_kernels/xSurfaceOdometry.h)

# Build Library
add_library(eval_engine ${SRC})

target_link_libraries(
  eval_engine
  ${MOOS_LIBRARIES}
  logutils
  ufield
  apputil
  mbutil
  m
  pthread)
