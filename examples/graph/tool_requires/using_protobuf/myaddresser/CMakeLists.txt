cmake_minimum_required(VERSION 3.15)
project(myaddresser LANGUAGES CXX)

find_package(protobuf CONFIG REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS addressbook.proto)


add_library(myaddresser src/myaddresser.cpp ${PROTO_SRCS})
target_include_directories(myaddresser PUBLIC include)

target_include_directories(myaddresser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include> 
)

target_link_libraries(myaddresser PUBLIC protobuf::libprotobuf)


set_target_properties(myaddresser PROPERTIES PUBLIC_HEADER "include/myaddresser.h;${PROTO_HDRS}")
install(TARGETS myaddresser)
