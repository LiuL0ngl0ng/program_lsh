cmake_minimum_required(VERSION 2.8.3)

project(opt_utils)

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  lanelet_map_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs lanelet_map_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/base.cpp
  src/circle.cpp
  src/pose2d.cpp
  src/state2d.cpp
  src/stringprintf.cpp
  src/path_property.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  glog)

catkin_add_gtest(csv_test
  tests/csv_test.cpp)

target_link_libraries(csv_test
  ${catkin_LIBRARIES})
