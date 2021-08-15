cmake_minimum_required(VERSION 2.8.3)

project(opt_solver)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(catkin_deps
  roscpp
  roslib
  nav_msgs
  opt_utils)
find_package(catkin REQUIRED COMPONENTS
  ${catkin_deps}
)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(Ceres REQUIRED)

find_package(IPOPT REQUIRED)

#find_package(SNOPT REQUIRED)

find_package(NLOPT REQUIRED)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()
add_definitions(${EIGEN_DEFINITIONS})



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${catkin_deps}
  DEPENDS CERES
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
#  ${SNOPT_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${IPOPT_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/quadratic_cost_function.cpp
  src/se2_constraints.cpp
  src/si_bending_energy.cpp
  src/nlp_solver.cpp
  src/ipopt_problem.cpp
  src/nlopt_problem.cpp
  src/jerk_constraints.cpp
  src/solution.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${IPOPT_LIBRARIES}
#  ${SNOPT_LIBRARIES}
  ${NLOPT_LIBRARIES}
  ${CERES_LIBRARIES})

catkin_add_gtest(quadratic_cost_function_test
  tests/quadratic_cost_function_test.cpp)

target_link_libraries(quadratic_cost_function_test
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PROJECT_NAME})

add_executable(single_path_node
  src/path_generator.cpp)

target_link_libraries(single_path_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${IPOPT_LIBRARIES}
  ${NLOPT_LIBRARIES}
#  ${SNOPT_LIBRARIES}
  ${PROJECT_NAME})


catkin_add_gtest(nlp_solver_test
  tests/nlp_solver_test.cpp)

target_link_libraries(nlp_solver_test
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${IPOPT_LIBRARIES}
  ${NLOPT_LIBRARIES}
#  ${SNOPT_LIBRARIES}
  ${PROJECT_NAME})
