cmake_minimum_required(VERSION 2.8.3)
project(gps_utils)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED  COMPONENTS
        roscpp
        sensor_msgs
        geometry_msgs)

include_directories(include
        ${catkin_INCLUDE_DIRS})
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES gps_utils
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/gps_utils)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/include/gps_utils/enu.h
        DESTINATION ${CATKIN_DEVEL_PREFIX}/include/gps_utils/)


add_library(gps_utils
        src/coord_system.c
        src/linear_algebra.c
        src/enu.cpp)

target_link_libraries(gps_utils
        ${catkin_LIBRARIES})


# Install gps_utils ros library.
install(TARGETS gps_utils
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install gps_utils ros header.
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
