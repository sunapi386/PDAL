###############################################################################
#
# CMake module to search for AutoXBF
#
# On success, the macro sets the following variables:
# AutoXBF_FOUND    = if the lib is found
# AutoXBF_RUNTIME  = full path to the AutoXBF lib
#
###############################################################################
MESSAGE(STATUS "Searching for AutoXBF")

IF(AUTOXBF_LIB)
  # Already in cache, be silent
  SET(AUTOXBF_FIND_QUIETLY TRUE)
ENDIF()

FIND_FILE(AUTOXBF_LIB
  libautoxbf.a # todo: once this file is found the .h files are not checked... fix this
  autox_macros.h
  autox_datum_parser.h
  autox_datum_writer.h
  autox_timespec_utils.h
  PATHS
  /usr/local/lib
  /usr/local/include
  /usr/local)

# Handle the QUIETLY and REQUIRED arguments and set AutoXBF_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(AutoXBF DEFAULT_MSG AUTOXBF_LIB)
