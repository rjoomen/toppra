# FindTesseract.cmake
# Finds the Tesseract motion planning framework components
#
# Usage:
#   find_package(Tesseract COMPONENTS environment kinematics urdf scene_graph)
#
# This will define:
#   Tesseract_FOUND - System has Tesseract
#   Tesseract_INCLUDE_DIRS - The Tesseract include directories
#   Tesseract_LIBRARIES - The libraries needed to use Tesseract
#   Tesseract_VERSION - The version of Tesseract found

# List of Tesseract components that can be found
set(Tesseract_COMPONENTS
  environment
  kinematics
  urdf
  scene_graph
  collision
  common
  srdf
)

# If no components specified, use all common ones
if(NOT Tesseract_FIND_COMPONENTS)
  set(Tesseract_FIND_COMPONENTS environment kinematics urdf scene_graph common)
endif()

set(Tesseract_FOUND TRUE)
set(Tesseract_LIBRARIES "")
set(Tesseract_INCLUDE_DIRS "")

# Find each requested component
foreach(component ${Tesseract_FIND_COMPONENTS})
  find_package(tesseract_${component} QUIET)

  if(tesseract_${component}_FOUND)
    message(STATUS "Found Tesseract component: ${component}")
    list(APPEND Tesseract_LIBRARIES tesseract::tesseract_${component})

    # Get include directories if available
    if(TARGET tesseract::tesseract_${component})
      get_target_property(_inc_dirs tesseract::tesseract_${component}
                          INTERFACE_INCLUDE_DIRECTORIES)
      if(_inc_dirs)
        list(APPEND Tesseract_INCLUDE_DIRS ${_inc_dirs})
      endif()
    endif()
  else()
    if(Tesseract_FIND_REQUIRED)
      set(Tesseract_FOUND FALSE)
      message(FATAL_ERROR "Required Tesseract component not found: ${component}")
    else()
      message(WARNING "Tesseract component not found: ${component}")
    endif()
  endif()
endforeach()

# Remove duplicates
if(Tesseract_INCLUDE_DIRS)
  list(REMOVE_DUPLICATES Tesseract_INCLUDE_DIRS)
endif()

# Try to find version
if(tesseract_common_FOUND)
  set(Tesseract_VERSION ${tesseract_common_VERSION})
endif()

if(Tesseract_FOUND)
  message(STATUS "Found Tesseract (version ${Tesseract_VERSION})")
  message(STATUS "  Libraries: ${Tesseract_LIBRARIES}")
endif()

mark_as_advanced(Tesseract_LIBRARIES Tesseract_INCLUDE_DIRS)
