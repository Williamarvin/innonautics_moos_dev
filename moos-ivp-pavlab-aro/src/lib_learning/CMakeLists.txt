#--------------------------------------------------------
# The CMakeLists.txt for:                       pDynamLearning
# Author(s):                              Tyler Paine
#--------------------------------------------------------

find_package(BLAS)
find_package(LAPACK)

if(BLAS_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  
  SET(SRC
    SimpleNN.cpp
    SimpleAID.cpp
    SimpleGPR.cpp
    SimpleConsensus.cpp
    SimpleKalmanConsensus.cpp
    )
  
  SET(HEADERS
    SimpleNN.h
    SimpleAID.h
    SimpleGPR.h
    SimpleRLS.h
    SimpleConsensus.h
    SimpleKalmanConsensus.h
    )
  
  # Build Library
  ADD_LIBRARY(learning ${SRC})
  
  add_definitions(-DARMA_DONT_USE_WRAPPER)
  
  target_include_directories(learning PRIVATE ../lib_armadillo/armadillo-12.6.6/include/)
  
  TARGET_LINK_LIBRARIES(learning
    ${BLAS_LIBRARIES}
    ${LAPACk_LIBRARIES}
    )
  
else()
  message(WARNING "BLAS library not found - Not Building <lib_learning> \n Try: \n\t - Mac: <brew install openblas> \n\t - Ubuntu/Debian: <sudo apt install libblas-dev liblapack-dev>")
endif()
