# - Try to find ZBar
# Once done this will define
#  ZBAR_FOUND - System has LibXml2
#  ZBAR_INCLUDE_DIRS - The LibXml2 include directories
#  ZBAR_LIBRARIES - The libraries needed to use LibXml2
#  ZBAR_DEFINITIONS - Compiler switches required for using LibXml2

find_package(PkgConfig)
pkg_check_modules(PC_ZBAR QUIET zbar)
set(ZBAR_DEFINITIONS ${PC_ZBAR_CFLAGS_OTHER})

find_path(ZBAR_INCLUDE_DIR Decoder.h
          HINTS ${PC_ZBAR_INCLUDEDIR} ${PC_ZBAR_INCLUDE_DIRS}
          PATH_SUFFIXES zbar)

find_library(ZBAR_LIBRARY NAMES zbar libzbar
             HINTS ${PC_ZBAR_LIBDIR} ${PC_ZBAR_LIBRARY_DIRS})

set(ZBAR_LIBRARIES ${ZBAR_LIBRARY})
set(ZBAR_INCLUDE_DIRS ${ZBAR_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZBAR DEFAULT_MSG ZBAR_INCLUDE_DIR ZBAR_LIBRARY)

mark_as_advanced(ZBAR_INCLUDE_DIR ZBAR_LIBRARY)
