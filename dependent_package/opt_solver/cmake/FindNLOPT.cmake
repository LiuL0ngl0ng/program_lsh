# Find NLOPT
#
# This sets the following variables:
# NLOPT_FOUND
# NLOPT_INCLUDE_DIRS
# NLOPT_LIBRARIES
# NLOPT_DEFINITIONS
# NLOPT_VERSION

find_package(PkgConfig)
pkg_check_modules(PC_NLOPT REQUIRED nlopt)
set(NLOPT_DEFINITIONS ${PC_NLOPT_CFLAGS_OTHER})
set(NLOPT_INCLUDE_DIRS ${PC_NLOPT_INCLUDE_DIRS})
set(NLOPT_LIBRARIES ${PC_NLOPT_LIBRARIES})
set(NLOPT_VERSION ${PC_NLOPT_VERSION})
include(FindPackageHandleStandardArgs)
# if all listed variables are TRUE
find_package_handle_standard_args(NLOPT DEFAULT_MSG
  NLOPT_LIBRARIES NLOPT_INCLUDE_DIRS)
mark_as_advanced(NLOPT_INCLUDE_DIRS NLOPT_LIBRARIES)

if(${NLOPT_FOUND})
  message(STATUS "Found NLOPT version: " ${NLOPT_VERSION} " installed in: " ${PC_NLOPT_PREFIX})
else()
  message(SEND_ERROR "Could not find NLOPT")
endif()