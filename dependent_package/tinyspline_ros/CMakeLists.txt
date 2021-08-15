cmake_minimum_required(VERSION 2.8.3)
project(tinyspline_ros)

## Add support for C++11, supported in ROS Kinetic and newer
#add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tinyspline_ros_cpp
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_c
  src/${PROJECT_NAME}/tinyspline.c
)
add_library(${PROJECT_NAME}_cpp
  src/${PROJECT_NAME}/tinysplinecpp.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_c ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_c
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_cpp
  ${PROJECT_NAME}_c
  ${catkin_LIBRARIES}
)


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
)


