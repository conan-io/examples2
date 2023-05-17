cmake_minimum_required(VERSION 3.15)
project(myserver LANGUAGES CXX)

find_package(protobuf CONFIG REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS addressbook.proto)


add_library(myserver src/myserver.cpp ${PROTO_SRCS})
target_include_directories(myserver PUBLIC include)

target_include_directories(myserver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include> 
)

target_link_libraries(myserver PUBLIC protobuf::libprotobuf)


set_target_properties(myserver PROPERTIES PUBLIC_HEADER "include/myserver.h;${PROTO_HDRS}")
install(TARGETS myserver)
