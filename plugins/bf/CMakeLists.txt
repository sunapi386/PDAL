#
# BF driver CMake configuration
#
set(CMAKE_CXX_STANDARD 14)
include(${PDAL_CMAKE_DIR}/bf.cmake)


# eigen lib
find_package(Eigen3 3.3.4 REQUIRED NO_MODULE)

# geographiclib
find_package(GeographicLib REQUIRED)

# protobuf lib & compile proto files
find_package(Protobuf REQUIRED)
include(FindProtobuf)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS io/vehicle_state.proto)

#add_executable(reader_bf ReaderBF.cc)
#message(BF ${BF})

# pthreads lib
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#target_link_libraries(reader_bf ${BF} Threads::Threads ${PROTOBUF_LIBRARIES} Eigen3::Eigen GeographicLib)

#
# BF Reader
#
PDAL_ADD_PLUGIN(reader_libname reader bf
                FILES
                io/BFReader.cpp
                io/BFCommon.cpp
                ${PROTO_SRCS}
                LINK_WITH
                ${BF}
                Threads::Threads ${PROTOBUF_LIBRARIES} Eigen3::Eigen GeographicLib
                INCLUDES
                ${PROTO_HDRS}
        )

#
# BF Writer
#
PDAL_ADD_PLUGIN(writer_libname writer bf
                FILES
                io/BFWriter.cpp
                io/BFCommon.cpp
                LINK_WITH
                ${BF})

