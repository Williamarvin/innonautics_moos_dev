#--------------------------------------------------------
# The CMakeLists.txt for:                       pDynamLearning
# Author(s):                              Tyler Paine
#--------------------------------------------------------

find_package(BLAS)
find_package(LAPACK)


if(BLAS_FOUND AND LAPACK_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

  SET(SRC
    DynamLearning.cpp
    DynamLearning_Info.cpp
    main.cpp
   )

 ADD_EXECUTABLE(pDynamLearning ${SRC})

 add_definitions(-DARMA_DONT_USE_WRAPPER)
 
 target_include_directories(pDynamLearning PRIVATE ../lib_armadillo/armadillo-12.6.6/include/)

 TARGET_LINK_LIBRARIES(pDynamLearning
   ${BLAS_LIBRARIES}
   ${LAPACK_LIBRARIES}
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread
   learning)


else()
    message(WARNING "BLAS or LAPACK library not found - Not Building <pDynamLearning> \n Try: \n\t - Mac: <brew install openblas> \n\t - Ubuntu/Debian: <sudo apt install libblas-dev liblapack-dev>")
endif()
