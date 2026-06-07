cmake_minimum_required(VERSION 3.22)
project(cccad_geometry_service LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)
find_package(OpenCASCADE REQUIRED)
find_package(Threads REQUIRED)

set(CCCAD_OCCT_COMPONENTS
  TKernel
  TKMath
  TKG2d
  TKG3d
  TKGeomBase
  TKBRep
  TKGeomAlgo
  TKTopAlgo
  TKPrim
  TKBO
  TKBool
  TKFillet
  TKMesh
)

set(CCCAD_OCCT_LIBRARIES)
foreach(component IN LISTS CCCAD_OCCT_COMPONENTS)
  find_library(CCCAD_OCCT_${component}_LIBRARY NAMES ${component} REQUIRED)
  list(APPEND CCCAD_OCCT_LIBRARIES ${CCCAD_OCCT_${component}_LIBRARY})
endforeach()

set(PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/proto/geometry_kernel.proto)
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})

set(GENERATED_PROTO_SRC ${GENERATED_DIR}/geometry_kernel.pb.cc)
set(GENERATED_PROTO_HDR ${GENERATED_DIR}/geometry_kernel.pb.h)
set(GENERATED_GRPC_SRC ${GENERATED_DIR}/geometry_kernel.grpc.pb.cc)
set(GENERATED_GRPC_HDR ${GENERATED_DIR}/geometry_kernel.grpc.pb.h)

get_target_property(GRPC_CPP_PLUGIN_EXECUTABLE gRPC::grpc_cpp_plugin LOCATION)

add_custom_command(
  OUTPUT ${GENERATED_PROTO_SRC} ${GENERATED_PROTO_HDR} ${GENERATED_GRPC_SRC} ${GENERATED_GRPC_HDR}
  COMMAND protobuf::protoc
  ARGS --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
       --cpp_out=${GENERATED_DIR}
       --grpc_out=${GENERATED_DIR}
       --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_EXECUTABLE}
       ${PROTO_FILE}
  DEPENDS ${PROTO_FILE}
  COMMENT "Generating protobuf and gRPC sources"
)

add_library(cccad_geometry_proto
  ${GENERATED_PROTO_SRC}
  ${GENERATED_GRPC_SRC}
)

target_include_directories(cccad_geometry_proto PUBLIC ${GENERATED_DIR})
target_link_libraries(cccad_geometry_proto PUBLIC protobuf::libprotobuf gRPC::grpc++)

add_executable(cccad-geometry-service
  src/main.cpp
  src/boolean_builder.cpp
  src/pattern_builder.cpp
  src/geometry_service.cpp
  src/edge_modifier_builder.cpp
  src/extrude_builder.cpp
  src/hole_builder.cpp
  src/artifact_writer.cpp
)

target_include_directories(cccad-geometry-service PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GENERATED_DIR}
  ${OpenCASCADE_INCLUDE_DIR}
)

target_link_libraries(cccad-geometry-service PRIVATE
  cccad_geometry_proto
  gRPC::grpc++
  protobuf::libprotobuf
  Threads::Threads
  ${CCCAD_OCCT_LIBRARIES}
)
