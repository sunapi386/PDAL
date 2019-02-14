#
# AutoXBF driver CMake configuration
#

include(${PDAL_CMAKE_DIR}/autoxbf.cmake)

#
# AutoXBF Reader
#
PDAL_ADD_PLUGIN(reader_libname reader autoxbf
    FILES
        io/AutoXBFReader.cpp
    LINK_WITH
        ${AUTOXBF})

#
# AutoXBF Writer
#
PDAL_ADD_PLUGIN(writer_libname writer autoxbf
    FILES
        io/AutoXBFWriter.cpp
    LINK_WITH
        ${AUTOXBF})

