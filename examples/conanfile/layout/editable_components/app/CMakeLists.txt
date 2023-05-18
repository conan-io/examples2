
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_CXX_ABI_COMPILED 1)
cmake_minimum_required(VERSION 3.0)
project(PackageTest CXX)

find_package(MYG)

add_executable(example example.cpp)
target_link_libraries(example MyGreetings::MyGreetings)

add_executable(example2 example2.cpp)
target_link_libraries(example2 MyGreetings::MyHello)
