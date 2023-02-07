cmake_minimum_required(VERSION 3.15)
project(say CXX)

add_library(say src/say.cpp)
target_include_directories(say PUBLIC include)

set_target_properties(say PROPERTIES PUBLIC_HEADER "include/say.h")

add_executable(hello src/hello.cpp)
target_link_libraries(hello say)

# The executables are generated at the "bin" folder
set_target_properties(hello PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

install(TARGETS say hello)

