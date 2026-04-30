# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mower_vfh_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mower_vfh_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mower_vfh_FOUND FALSE)
  elseif(NOT mower_vfh_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mower_vfh_FOUND FALSE)
  endif()
  return()
endif()
set(_mower_vfh_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mower_vfh_FIND_QUIETLY)
  message(STATUS "Found mower_vfh: 0.1.0 (${mower_vfh_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mower_vfh' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mower_vfh_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mower_vfh_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mower_vfh_DIR}/${_extra}")
endforeach()
