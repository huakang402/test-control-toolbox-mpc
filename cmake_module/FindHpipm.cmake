# - Find hpipm library
# Find the hpipm headers and libraries.
#
#  HPIPM_INCLUDE_DIRS - where to find hpipm.h
#  HPIPM_LIBRARIES    - List of libraries when using hpipm.
#  HPIPM_FOUND        - True if hpipm library found.
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

set(HPIPM_ROOT_DIR
    "${HPIPM_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for hpipm")

# Look for the header file.
find_path(HPIPM_INCLUDE_DIR
    NAMES
    hpipm_d_ocp_qp.h
    HINTS
    "${HPIPM_ROOT_DIR}"
    PATH_SUFFIXES
    include
    PATHS
    /opt/hpipm
    /opt/hpipm/include
    /usr/local
    /usr/include/hpipm
    /usr/local/include/hpipm)

# Look for the library.
find_library(HPIPM_LIBRARY
    NAMES
    libhpipm.a
    hpipm.a
    hpipm
    PATH_SUFFIXES
    lib
    PATHS
    /usr
    /usr/local)

message(STATUS "Here" ${HPIPM_LIBRARY})

# handle the QUIETLY and REQUIRED arguments and set BLASFEO_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(hpipm
    DEFAULT_MSG
    HPIPM_LIBRARY
    HPIPM_INCLUDE_DIR)

if (HPIPM_FOUND)
  set(HPIPM_LIBRARIES ${HPIPM_LIBRARY})
  set(HPIPM_INCLUDE_DIRS ${HPIPM_INCLUDE_DIR})

  mark_as_advanced(HPIPM_ROOT_DIR)
else ()
  set(HPIPM_LIBRARIES)
  set(HPIPM_INCLUDE_DIRS)
endif ()

mark_as_advanced(HPIPM_LIBRARY HPIPM_INCLUDE_DIR)