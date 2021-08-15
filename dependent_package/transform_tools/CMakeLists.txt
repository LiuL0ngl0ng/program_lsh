cmake_minimum_required(VERSION 2.8.3)
project(transform_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(catkin_deps
  roscpp
  geometry_msgs
  tf)
find_package(catkin REQUIRED COMPONENTS
  ${catkin_deps}
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}_lib
   CATKIN_DEPENDS ${catkin_deps}
   DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}_lib
   src/transform.cc
   src/rigid_transform.cc
   src/transform_interface.cpp
 )

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_lib
   ${catkin_LIBRARIES}
   ${EIGEN3_LIBRARIES}
 )

