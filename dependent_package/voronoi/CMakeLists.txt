cmake_minimum_required(VERSION 2.8.3)
project(voronoi)
#set(CMAKE_BUILD_TYPE "Release")
## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
set(catkin_deps
  roscpp
  internal_grid_map
  grid_map_sdf)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${catkin_deps})
find_package(benchmark REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include/voronoi
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS ${catkin_deps}
  DEPENDS)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})


## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/voronoi_diagram.cpp
  include/voronoi/voronoi_diagram.hpp)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  glog)

add_executable(voronoi_benchmark
  test/voronoi_benchmark.cpp)
target_link_libraries(voronoi_benchmark
  ${PROJECT_NAME}_lib
  benchmark::benchmark)

add_executable(voronoi_function_test
  test/test_voronoi_function.cpp)
target_link_libraries(voronoi_function_test
  ${PROJECT_NAME}_lib)

add_executable(voronoi_test_node
  test/voroni_node.cpp)
target_link_libraries(voronoi_test_node
  ${PROJECT_NAME}_lib)

