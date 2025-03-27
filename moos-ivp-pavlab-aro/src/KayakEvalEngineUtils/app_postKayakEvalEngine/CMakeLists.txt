# --------------------------------------------------------
# The CMakeLists.txt for:           EvalEngine - Kayak V0 Author(s): Ray Turrisi
# --------------------------------------------------------

# Set System Specific Libraries
if(${WIN32})
  set(SYSTEM_LIBS wsock32)
else(${WIN32})
  set(SYSTEM_LIBS m)
endif(${WIN32})

set(SRC main.cpp)

add_executable(postKayakEvalEngine ${SRC})

target_link_libraries(
  postKayakEvalEngine
  logutils
  ${SYSTEM_LIBS}
  ${MOOS_LIBRARIES}
  MOOSGeodesy
  apputil
  logutils
  mbutil
  m
  eval_engine)
