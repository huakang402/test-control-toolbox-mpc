# - Find blasfeo library
# Find the blasfeo headers and libraries.
#
#  BLASFEO_INCLUDE_DIRS - where to find blasfeo.h
#  BLASFEO_LIBRARIES    - List of libraries when using blasfeo.
#  BLASFEO_FOUND        - True if blasfeo library found.
#

# Based on Findquatlib.cmake, Original Author:
# 2009-2010 Ryan Pavlik <rpavlik@iastate.edu> <abiryan@ryand.net>
# http://academic.cleardefinition.com
# Iowa State University HCI Graduate Program/VRAC
#
# Copyright Iowa State University 2009-2010.
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)

set(BLASFEO_ROOT_DIR
    "${BLASFEO_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for blasfeo")

# Look for the header file.
find_path(BLASFEO_INCLUDE_DIR
    NAMES
    blasfeo_target.h
    HINTS
    "${BLASFEO_ROOT_DIR}"
    PATH_SUFFIXES
    include
    PATHS
    /opt/blasfeo
    /opt/blasfeo/include
    /usr/local
    /usr/include/blasfeo
    /usr/local/include/blasfeo)

# Look for the library.
find_library(BLASFEO_LIBRARY
    NAMES
    libblasfeo.a
    blasfeo.a
    blasfeo
    PATH_SUFFIXES
    lib
    PATHS
    /usr
    /usr/local)

message(STATUS "Here" ${BLASFEO_LIBRARY})

# handle the QUIETLY and REQUIRED arguments and set BLASFEO_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(blasfeo
    DEFAULT_MSG
    BLASFEO_LIBRARY
    BLASFEO_INCLUDE_DIR)

if (BLASFEO_FOUND)
  set(BLASFEO_LIBRARIES ${BLASFEO_LIBRARY})
  set(BLASFEO_INCLUDE_DIRS ${BLASFEO_INCLUDE_DIR})

  mark_as_advanced(BLASFEO_ROOT_DIR)
else ()
  set(BLASFEO_LIBRARIES)
  set(BLASFEO_INCLUDE_DIRS)
endif ()

mark_as_advanced(BLASFEO_LIBRARY BLASFEO_INCLUDE_DIR)