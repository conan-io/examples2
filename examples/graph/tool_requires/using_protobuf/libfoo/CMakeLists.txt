cmake_minimum_required(VERSION 3.15)
project(libfoo LANGUAGES CXX)

find_package(Protobuf CONFIG REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS addressbook.proto)


add_library(libfoo src/libfoo.cpp ${PROTO_SRCS})
target_include_directories(libfoo PUBLIC include)

target_include_directories(libfoo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include> 
)

target_link_libraries(libfoo PUBLIC protobuf::libprotobuf)


set_target_properties(libfoo PROPERTIES PUBLIC_HEADER "include/libfoo.h;${PROTO_HDRS}")
install(TARGETS libfoo)
