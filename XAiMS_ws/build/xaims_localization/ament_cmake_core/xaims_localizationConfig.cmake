# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_xaims_localization_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED xaims_localization_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(xaims_localization_FOUND FALSE)
  elseif(NOT xaims_localization_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(xaims_localization_FOUND FALSE)
  endif()
  return()
endif()
set(_xaims_localization_CONFIG_INCLUDED TRUE)

# output package information
if(NOT xaims_localization_FIND_QUIETLY)
  message(STATUS "Found xaims_localization: 0.0.0 (${xaims_localization_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'xaims_localization' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${xaims_localization_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(xaims_localization_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${xaims_localization_DIR}/${_extra}")
endforeach()
