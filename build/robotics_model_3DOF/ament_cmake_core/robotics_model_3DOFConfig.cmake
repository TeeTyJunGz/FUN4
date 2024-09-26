# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robotics_model_3DOF_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robotics_model_3DOF_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robotics_model_3DOF_FOUND FALSE)
  elseif(NOT robotics_model_3DOF_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robotics_model_3DOF_FOUND FALSE)
  endif()
  return()
endif()
set(_robotics_model_3DOF_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robotics_model_3DOF_FIND_QUIETLY)
  message(STATUS "Found robotics_model_3DOF: 0.0.0 (${robotics_model_3DOF_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robotics_model_3DOF' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${robotics_model_3DOF_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robotics_model_3DOF_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${robotics_model_3DOF_DIR}/${_extra}")
endforeach()
