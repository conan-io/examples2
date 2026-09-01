cmake_minimum_required(VERSION 3.15)
project(sycl_app CXX)

find_package(ZLIB REQUIRED)
find_package(fmt REQUIRED)

add_executable(sycl_app src/main.cpp)
target_link_libraries(sycl_app PRIVATE ZLIB::ZLIB fmt::fmt)
target_compile_features(sycl_app PRIVATE cxx_std_17)
