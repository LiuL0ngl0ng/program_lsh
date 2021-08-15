cmake_minimum_required(VERSION 2.8.3)
project(movobsplan_msgs)

set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  message_generation
  std_msgs
  geometry_msgs
)

