cmake_minimum_required(VERSION 3.15)
project(compressor C)

find_package(ZLIB REQUIRED)


add_executable(${PROJECT_NAME} src/main.c)
target_link_libraries(${PROJECT_NAME} ZLIB::ZLIB)

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS "CMAKE_VER=\"${CMAKE_VERSION}\"")
