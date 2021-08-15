cmake_minimum_required(VERSION 2.8.3)

project(car_model)

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")

set(catkin_deps
  opt_utils
  roscpp)

find_package(catkin REQUIRED COMPONENTS
  ${catkin_deps}
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${catkin_deps}
  #  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/car_geometry.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES})



catkin_add_gtest(car
  tests/car_geometry_test.cpp)

target_link_libraries(car
  ${catkin_LIBRARIES}
  ${PROJECT_NAME})


