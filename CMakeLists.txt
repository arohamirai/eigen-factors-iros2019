cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)

project(EigenFactors)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# ===================================================================
# DEPENDENCIES: locate the necessary local dependencies for the project
# include as dependency TODO include xternal Eigen?
find_package(Eigen QUIET)
if(Eigen_FOUND)
  MESSAGE("Eigen found in CMake modules:" ${Eigen_INCLUDE_DIRS})
ELSE(Eigen_FOUND)  
    SET(Eigen_INCLUDE_DIRS /usr/local/include/eigen3)
    MESSAGE("Eigen not found, adding manually directories. BE CAREFULL to have installed Eigen here:" ${Eigen_INCLUDE_DIRS})
    MESSAGE(${Eigen_INCLUDE_DIRS})
ENDIF(Eigen_FOUND)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})


# MROB modules. Remove those that are not necessary, by default all are active
INCLUDE_DIRECTORIES(./mrob/src/common)

INCLUDE_DIRECTORIES(./mrob/src/SE3)
ADD_SUBDIRECTORY(./mrob/src/SE3)

INCLUDE_DIRECTORIES(./mrob/src/PCRegistration)
ADD_SUBDIRECTORY(./mrob/src/PCRegistration)


# Find installed Open3D, this imports
#   Open3D_INCLUDE_DIRS
#   Open3D_LIBRARIES
#   Open3D_LIBRARY_DIRS
#   Open3D_C_FLAGS
#   Open3D_CXX_FLAGS
#   Open3D_EXE_LINKER_FLAGS
if(WIN32)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
else()
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
    list(APPEND Open3D_LIBRARIES dl)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

# Set OS-specific things here
if(WIN32)
elseif(CYGWIN)
elseif(APPLE)
elseif(UNIX)
	add_definitions(-DUNIX)
	add_compile_options(-Wno-deprecated-declarations)
	add_compile_options(-Wno-unused-result)
    add_definitions(-O3)
endif(WIN32)




# Open3D
message(STATUS "Found Open3D ${Open3D_VERSION}")


# link_directories must be before add_executable
link_directories(${Open3D_LIBRARY_DIRS})
add_executable(planeVisualizer planeVisualizer.cpp)
target_link_libraries(planeVisualizer ${Open3D_LIBRARIES} PCRegistration SE3)
target_include_directories(planeVisualizer PUBLIC ${Open3D_INCLUDE_DIRS})
# Hot fix windows dll not found issue, assumming we're using the Release build
option(BUILD_SHARED_LIBS "Whether Open3D was build as shared library" OFF)


# evaluation of alpha and beta parameters for tuning
add_executable(syntheticOptimizationEval_params syntheticOptimizationEval_params.cpp)
target_link_libraries(syntheticOptimizationEval_params PCRegistration SE3)

# evaluation of NAG vs gradient descent, sanity check on iteres and error
add_executable(syntheticOptimizationEval_nag_vs_gd syntheticOptimizationEval_nag_vs_gd.cpp)
target_link_libraries(syntheticOptimizationEval_nag_vs_gd PCRegistration SE3)


# evaluation wrt ICP p2p and p2plane
add_executable(syntheticOptimizationEval_eigen_vs_icps syntheticOptimizationEval_eigen_vs_icps.cpp)
target_link_libraries(syntheticOptimizationEval_eigen_vs_icps ${Open3D_LIBRARIES} PCRegistration SE3)
target_include_directories(syntheticOptimizationEval_eigen_vs_icps PUBLIC ${Open3D_INCLUDE_DIRS})


# evaluation wrt ICP p2p and p2plane and INitial error
#add_executable(syntheticOptimizationEval_initial_error syntheticOptimizationEval_initial_error.cpp)
#target_link_libraries(syntheticOptimizationEval_initial_error ${Open3D_LIBRARIES} ${PCRegistration_LIBRARIES} ${SE3_LIBRARIES})
#target_include_directories(syntheticOptimizationEval_initial_error PUBLIC ${Open3D_INCLUDE_DIRS})
