# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fsm_pilot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fsm_pilot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fsm_pilot_FOUND FALSE)
  elseif(NOT fsm_pilot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fsm_pilot_FOUND FALSE)
  endif()
  return()
endif()
set(_fsm_pilot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fsm_pilot_FIND_QUIETLY)
  message(STATUS "Found fsm_pilot: 1.1.0 (${fsm_pilot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fsm_pilot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fsm_pilot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fsm_pilot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fsm_pilot_DIR}/${_extra}")
endforeach()
