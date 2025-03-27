#--------------------------------------------------------
# The CMakeLists.txt for:                  test_armadillo
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------



#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_CXX_FLAGS "-O3")

find_package(BLAS)

if(BLAS_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  FILE(GLOB SRC *.cpp)

  ADD_EXECUTABLE(test_armadillo 
    main.cpp)

  add_definitions(-DARMA_DONT_USE_WRAPPER)

  target_include_directories(test_armadillo PRIVATE ../lib_armadillo/armadillo-12.6.6/include/)

  TARGET_LINK_LIBRARIES(test_armadillo
    ${BLAS_LIBRARIES}
    ${SYSTEM_LIBS}
  )

else()
    message(WARNING "BLAS library not found - Not Building <test_armadillo> \n Try: \n\t - Mac: <brew install openblas> \n\t - Ubuntu/Debian: <sudo apt install libblas-dev liblapack-dev>")
endif()
