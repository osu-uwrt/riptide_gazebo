# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_uwrt_ros_gz_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED uwrt_ros_gz_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(uwrt_ros_gz_FOUND FALSE)
  elseif(NOT uwrt_ros_gz_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(uwrt_ros_gz_FOUND FALSE)
  endif()
  return()
endif()
set(_uwrt_ros_gz_CONFIG_INCLUDED TRUE)

# output package information
if(NOT uwrt_ros_gz_FIND_QUIETLY)
  message(STATUS "Found uwrt_ros_gz: 0.0.0 (${uwrt_ros_gz_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'uwrt_ros_gz' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${uwrt_ros_gz_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(uwrt_ros_gz_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${uwrt_ros_gz_DIR}/${_extra}")
endforeach()
