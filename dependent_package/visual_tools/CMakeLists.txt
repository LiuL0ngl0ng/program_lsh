cmake_minimum_required(VERSION 2.8.3)

project(visual_tools)

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)


set(PKG_DEPS
  roscpp
  roslib
  tf2_eigen
  tf2)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PKG_DEPS}
  DEPENDS)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS})

# define a sugar variable to collect all the libraries to link against
add_library(${PROJECT_NAME}
  src/visual_tools.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES})

add_executable(demo_node
  src/demo_node.cpp
  src/display_node.cpp)

target_link_libraries(demo_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})
