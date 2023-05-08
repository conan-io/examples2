cmake_minimum_required(VERSION 3.15)
project(compressor C)

message(STATUS "CMAKE VERSION=${CMAKE_VERSION}")

find_package(ZLIB REQUIRED)

add_executable(${PROJECT_NAME} src/main.c)
target_link_libraries(${PROJECT_NAME} ZLIB::ZLIB)
