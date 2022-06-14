cmake_minimum_required(VERSION 3.15)
project(PackageTest CXX)

find_package(GTest REQUIRED CONFIG)

add_executable(test_sum test.cpp)
target_link_libraries(test_sum GTest::gtest GTest::gtest_main)
