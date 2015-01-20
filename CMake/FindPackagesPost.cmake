
# Copyright (c) 2013-2014 Stefan Eilemann <eile@eyescale.ch>

# additional compile-time definitions

list(APPEND FIND_PACKAGES_DEFINES GLEW_MX) # always define GLEW_MX
list(APPEND FIND_PACKAGES_DEFINES GLEW_NO_GLU)
if(GLEW_MX_FOUND)
  list(APPEND FIND_PACKAGES_DEFINES EQ_FOUND_GLEW_MX)
else()
  list(APPEND FIND_PACKAGES_DEFINES GLEW_STATIC EQ_GLEW_INTERNAL)
  if(X11_FOUND AND APPLE)
    list(APPEND FIND_PACKAGES_DEFINES GLEW_APPLE_GLX)
  endif()
endif()

if(WIN32)
  list(APPEND FIND_PACKAGES_DEFINES WGL) # deprecated
  list(APPEND FIND_PACKAGES_DEFINES EQ_WGL_USED)
endif()

if(X11_FOUND)
  set(EQ_GLX_USED 1)
endif()

if(Qt4_FOUND OR QT4_FOUND AND QTVERSION VERSION_GREATER 4.6)
  set(EQ_QT_USED 1)
else()
  set(QT4_FOUND)
  set(Qt4_FOUND)
endif()

if(APPLE)
  if(CMAKE_OSX_ARCHITECTURES MATCHES "64")
    set(CUDA_FOUND 0)
    message(STATUS "  Disable CUDA due to missing 64 bit libraries")
  else()
    set(CUDA_64_BIT_DEVICE_CODE OFF)
  endif()
  foreach(ARCH ${CMAKE_OSX_ARCHITECTURES})
    if(ARCH STREQUAL "ppc" OR ARCH STREQUAL "i386")
      set(EQ_AGL_USED 1)
      list(APPEND EQ_EXAMPLES_OSX_ARCHITECTURES ${ARCH})
    endif()
  endforeach()
  if(NOT EQ_EXAMPLES_OSX_ARCHITECTURES)
    set(EQ_EXAMPLES_OSX_ARCHITECTURES ${CMAKE_OSX_ARCHITECTURES})
  endif()
  list(REMOVE_DUPLICATES EQ_EXAMPLES_OSX_ARCHITECTURES)
endif()
include(EqGLLibraries)

if(EQ_GLX_USED)
  list(APPEND FIND_PACKAGES_DEFINES GLX) # deprecated
  list(APPEND FIND_PACKAGES_DEFINES EQ_GLX_USED)
endif()
if(EQ_AGL_USED)
  list(APPEND FIND_PACKAGES_DEFINES AGL) # deprecated
  list(APPEND FIND_PACKAGES_DEFINES EQ_AGL_USED)
endif()
if(EQ_QT_USED)
  list(APPEND FIND_PACKAGES_DEFINES QT)
endif()
if(MAGELLAN_FOUND AND NOT EQ_AGL_USED)
  list(APPEND FIND_PACKAGES_DEFINES EQUALIZER_USE_MAGELLAN_GLX)
endif()

if(NOT EQUALIZER_USE_OSG)
  set(OPENSCENEGRAPH_FOUND)
endif()

if(NOT EQUALIZER_USE_HWLOC)
  set(HWLOC_FOUND)
  set(HWLOC_GL_FOUND)
endif()
if(HWLOC_GL_FOUND)
  list(APPEND FIND_PACKAGES_DEFINES EQUALIZER_USE_HWLOC_GL)
endif()
