cmake_minimum_required(VERSION 3.15)
project(ascii_image)

set(CMAKE_CXX_STANDARD 17)

find_package(CURL)
find_package(stb)

add_executable(ascii_image main.cpp)

target_link_libraries(ascii_image CURL::libcurl stb::stb)
