# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_multirotor_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED multirotor_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(multirotor_FOUND FALSE)
  elseif(NOT multirotor_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(multirotor_FOUND FALSE)
  endif()
  return()
endif()
set(_multirotor_CONFIG_INCLUDED TRUE)

# output package information
if(NOT multirotor_FIND_QUIETLY)
  message(STATUS "Found multirotor: 0.0.0 (${multirotor_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'multirotor' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${multirotor_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(multirotor_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${multirotor_DIR}/${_extra}")
endforeach()
