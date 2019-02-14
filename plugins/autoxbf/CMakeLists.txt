#
# AutoXBF driver CMake configuration
#

set(AUTOXBF_LIBRARIES "")
set(objs "")
include(${PDAL_CMAKE_DIR}/autoxbf.cmake)

#
# AutoXBF Reader
#
set(srcs io/AutoXBFReader.cpp)
set(incs io/AutoXBFReader.hpp)
PDAL_ADD_PLUGIN(reader_libname reader autoxbf
    FILES "${srcs}" "${incs}" 
    LINK_WITH ${AUTOXBF_LIBRARIES})

#
# AutoXBF Writer
#
set(srcs io/AutoXBFWriter.cpp)
set(incs io/AutoXBFWriter.hpp)
PDAL_ADD_PLUGIN(writer_libname writer autoxbf
    FILES "${srcs}" "${incs}" 
    LINK_WITH ${AUTOXBF_LIBRARIES})

