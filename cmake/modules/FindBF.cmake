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

IF(BF)
  # Already in cache, be silent
  SET(BF_FIND_QUIETLY TRUE)
ENDIF()

FIND_FILE(BF
        libbf.a # todo: once this file is found the .h files are not checked... fix this
        bf_macros.h
        bf_datum_parser.h
        bf_datum_writer.h
        bf_timespec_utils.h
        PATHS
        /usr/local/lib
        /usr/local/include
        /usr/local)

# Handle the QUIETLY and REQUIRED arguments and set BF_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(BF DEFAULT_MSG BF_LIB)
