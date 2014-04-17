cmake_minimum_required(VERSION 2.4.6)
PROJECT("Libsdf")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

SET( CMAKE_VERBOSE_MAKEFILE ON)

################################################################################
# Release by default
# Turn on Debug with "-DCMAKE_BUILD_TYPE=Debug"
IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
ENDIF()

IF( CMAKE_COMPILER_IS_GNUCXX )
   SET(CMAKE_CXX_FLAGS "-std=c++11 -Wno-deprecated")
   SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
   SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp -march=native -Wno-deprecated")
 
   ADD_DEFINITIONS("-Wall")
ENDIF()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Create variables used for exporting (from build tree) in LibsdfConfig.cmake
SET( Libsdf_LIBRARIES "" )
SET( Libsdf_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include )

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

# Export package for use from the build tree
EXPORT( PACKAGE Libsdf )

# Create the LibsdfConfig.cmake file for other cmake projects.
# ... for the build tree
SET( CONFIG_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET( CONFIG_DIR ${CMAKE_CURRENT_BINARY_DIR})
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/LibsdfConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/LibsdfConfig.cmake @ONLY )
# ... for the install tree
SET( CMAKECONFIG_INSTALL_DIR lib/cmake/Libsdf )
FILE( RELATIVE_PATH REL_INCLUDE_DIR
    "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
    "${CMAKE_INSTALL_PREFIX}/include" )

SET( Sophus_INCLUDE_DIR "\${Libsdf_CMAKE_DIR}/${REL_INCLUDE_DIR}" )
SET( CONFIG_SOURCE_DIR )
SET( CONFIG_DIR )
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/LibsdfConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/LibsdfConfig.cmake @ONLY )

INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/LibsdfConfig.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR} )

# Install headers
INSTALL(DIRECTORY sophus DESTINATION ${CMAKE_INSTALL_PREFIX}/include
        FILES_MATCHING PATTERN "*.h" )


