#.rst:
# FindBullet
# ----------
#
# Try to find the Bullet3 physics engine
#
#
#
# ::
#
#   This module defines the following variables
#
#
#
# ::
#
#   BULLET_FOUND - Was bullet3 found
#   BULLET_INCLUDE_DIRS - the Bullet3 include directories
#   BULLET_LIBRARIES - Link to this, by default it includes
#                      all bullet3 components (Dynamics,
#                      Collision, LinearMath, & SoftBody)
#
#
#
# ::
#
#   This module accepts the following variables
#
#
#
# ::
#
#   BULLET3_ROOT - Can be set to bullet3 install path or Windows build path

#=============================================================================
# Copyright 2009 Kitware, Inc.
# Copyright 2009 Philip Lowman <philip@yhbt.com>
# Modified 2015 by Mirs King <mirsking@gmail.com>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

macro(_FIND_BULLET3_LIBRARY _var)
  find_library(${_var}
     NAMES
        ${ARGN}
     HINTS
	${BULLET3_ROOT}/bin
  )
  mark_as_advanced(${_var})
endmacro()

macro(_BULLET3_APPEND_LIBRARIES _list _release)
   set(_debug ${_release}_DEBUG)
   if(${_debug})
      set(${_list} ${${_list}} optimized ${${_release}} debug ${${_debug}})
   else()
      set(${_list} ${${_list}} ${${_release}})
   endif()
endmacro()

find_path(BULLET3_INCLUDE_DIR NAMES btBulletCollisionCommon.h
  HINTS
    ${BULLET3_ROOT}/src
)

# Find the libraries

_FIND_BULLET3_LIBRARY(BULLET3_COMMON_LIBRARY          libBullet3Common_gmake_x64_release.a)
_FIND_BULLET3_LIBRARY(BULLET3_GEOMETRY_LIBRARY        libBullet3Geometry_gmake_x64_release.a)
_FIND_BULLET3_LIBRARY(BULLET3_DYNAMICS_LIBRARY        libBullet3Dynamics_gmake_x64_release.a)
_FIND_BULLET3_LIBRARY(BULLET3_COLLISION_LIBRARY       libBullet3Collision_gmake_x64_release.a)
_FIND_BULLET3_LIBRARY(BULLET3_OPENCL_LIBRARY          libBullet3OpenCL_clew_gmake_x64_release.a)


FIND_PACKAGE_HANDLE_STANDARD_ARGS(Bullet3 DEFAULT_MSG
    BULLET3_COMMON_LIBRARY BULLET3_GEOMETRY_LIBRARY
    BULLET3_DYNAMICS_LIBRARY BULLET3_COLLISION_LIBRARY 
    BULLET3_OPENCL_LIBRARY
    BULLET3_INCLUDE_DIR)

set(BULLET3_INCLUDE_DIRS ${BULLET3_INCLUDE_DIR})
if(BULLET3_FOUND)
   _BULLET3_APPEND_LIBRARIES(BULLET3_LIBRARIES BULLET3_COMMON_LIBRARY)
   _BULLET3_APPEND_LIBRARIES(BULLET3_LIBRARIES BULLET3_GEOMETRY_LIBRARY)
   _BULLET3_APPEND_LIBRARIES(BULLET3_LIBRARIES BULLET3_DYNAMICS_LIBRARY)
   _BULLET3_APPEND_LIBRARIES(BULLET3_LIBRARIES BULLET3_COLLISION_LIBRARY)
   _BULLET3_APPEND_LIBRARIES(BULLET3_LIBRARIES BULLET3_OPENCL_LIBRARY)
endif()
