# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_autoronto_app_task_ima_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED autoronto_app_task_ima_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(autoronto_app_task_ima_FOUND FALSE)
  elseif(NOT autoronto_app_task_ima_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(autoronto_app_task_ima_FOUND FALSE)
  endif()
  return()
endif()
set(_autoronto_app_task_ima_CONFIG_INCLUDED TRUE)

# output package information
if(NOT autoronto_app_task_ima_FIND_QUIETLY)
  message(STATUS "Found autoronto_app_task_ima: 0.0.0 (${autoronto_app_task_ima_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'autoronto_app_task_ima' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${autoronto_app_task_ima_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(autoronto_app_task_ima_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${autoronto_app_task_ima_DIR}/${_extra}")
endforeach()
