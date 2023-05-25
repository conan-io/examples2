cmake_minimum_required(VERSION 3.15)
project(sensor CXX)

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(Protobuf REQUIRED CONFIG)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS sensor.proto)
protobuf_generate_python(PROTO_PYS sensor.proto)

add_executable(${PROJECT_NAME} main.cc ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME} PUBLIC protobuf::libprotobuf)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_BINARY_DIR})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)

add_custom_target(proto_python ALL DEPENDS ${PROTO_PYS})
