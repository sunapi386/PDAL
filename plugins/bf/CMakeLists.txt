#
# BF driver CMake configuration
#
set(CMAKE_CXX_STANDARD 14)
include(${PDAL_CMAKE_DIR}/bf.cmake)

#
# BF Reader
#
PDAL_ADD_PLUGIN(reader_libname reader bf
                FILES
                io/BFReader.cpp
                io/BFCommon.cpp
                LINK_WITH
                ${BF})

#
# BF Writer
#
PDAL_ADD_PLUGIN(writer_libname writer bf
                FILES
                io/BFWriter.cpp
                io/BFCommon.cpp
                LINK_WITH
                ${BF})

