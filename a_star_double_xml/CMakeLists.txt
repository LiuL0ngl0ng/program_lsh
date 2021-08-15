cmake_minimum_required(VERSION 2.8.0)
project(a_star_double_xml)
set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")
set(catkin_deps
  qt_build
  roscpp
  std_msgs
  sensor_driver_msgs
  anm_msgs
  lanelet_map_msgs
  roslib
  extractroad_msg
  plan2control_msgs
  vertical_wall_grid
  transform_tools
  stiff_msgs
  tinyspline_ros
  three_one_msgs
  control_msgs
  )
find_package(catkin REQUIRED COMPONENTS
  ${catkin_deps}
)
#set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(OpenCV 2.4 REQUIRED)
find_package(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${catkin_deps}
#  DEPENDS OpenCV
)

add_definitions(-DQT_NO_KEYWORDS)
add_definitions(${QT_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(a_star_double_xml
  ${QT_SOURCES}
  ${QT_RESOURCES_CPP}
  ${QT_FORMS_HPP}
  ${QT_MOC_HPP}
  ${MOC_FILES}
 src/astardoublexml.cpp
  src/thread.cpp)

target_link_libraries(a_star_double_xml
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  Qt4::QtXml)
install(TARGETS a_star_double_xml RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

