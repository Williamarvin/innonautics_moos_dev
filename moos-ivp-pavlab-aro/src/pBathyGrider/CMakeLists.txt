#--------------------------------------------------------
# The CMakeLists.txt for:                       pBathyGrider
# Author(s):                              
#--------------------------------------------------------

find_package(BLAS)
find_package(LAPACK)

if(BLAS_FOUND AND LAPACK_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


  SET(SRC
    BathyGrider.cpp
    BathyGrider_Info.cpp
    main.cpp
    )

  ADD_EXECUTABLE(pBathyGrider ${SRC})
  
  
  add_definitions(-DARMA_DONT_USE_WRAPPER)
  
  target_include_directories(pBathyGrider PRIVATE ../lib_armadillo/armadillo-12.6.6/include/)
  
  TARGET_LINK_LIBRARIES(pBathyGrider
    ${BLAS_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${MOOS_LIBRARIES}
    MOOSGeodesy
    geometry
    apputil
    mbutil
    m
    pthread
    bathygrid
    learning
    )
  
  
else()
  message(WARNING "BLAS or LAPACK library not found - Not Building <pBathyGridder> \n Try: \n\t - Mac: <brew install openblas> \n\t - Ubuntu/Debian: <sudo apt install libblas-dev liblapack-dev>")
endif()
