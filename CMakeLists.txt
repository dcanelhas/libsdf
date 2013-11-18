cmake_minimum_required(VERSION 2.4.6)
SET(PROJECT_NAME libsdf)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

# SET(CMAKE_BUILD_TYPE Coverage)
SET(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "-O3 -fopenmp -std=c++11 -Wno-deprecated")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp -std=c++11 -mtune=native -march=native -Wno-deprecated")

# Eigen3 library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS})

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(sdf 
  src/sdf.cpp)

target_link_libraries(sdf
  ${OpenCV_LIBS}
)  

add_executable(demo_app
  src/main.cpp )

target_link_libraries(demo_app
  sdf
)
