cmake_minimum_required(VERSION 3.8)
project(glog_test)

set(CMAKE_CXX_STANDARD 14)

find_package(glog REQUIRED)

add_executable (myapp main.cpp)
target_link_libraries (myapp glog::glog)

install(TARGETS myapp)

