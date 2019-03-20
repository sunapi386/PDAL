###############################################################################
#
# CMake module to search for BF
#
# On success, the macro sets the following variables:
# BF_FOUND    = if the lib is found
# BF_RUNTIME  = full path to the BF lib
#
###############################################################################
MESSAGE(STATUS "Searching for BF")

find_path(BF_INCLUDE_DIR NAMES bf.hpp
        PATH_SUFFIXES "include")
mark_as_advanced(BF_INCLUDE_DIR)

find_library(BF_LIBRARY NAMES bf)
mark_as_advanced(BF_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(BF
        REQUIRED_VARS BF_LIBRARY BF_INCLUDE_DIR)

if (BF_FOUND)
  set(BF_LIBRARIES ${BF_LIBRARY})
  set(BF_INCLUDE_DIRS ${BF_INCLUDE_DIR})
else()
  message(WARNING "BF not found")
endif()
