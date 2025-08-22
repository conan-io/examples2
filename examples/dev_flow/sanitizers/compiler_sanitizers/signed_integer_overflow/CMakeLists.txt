cmake_minimum_required(VERSION 3.15)
project(signed_integer_overflow LANGUAGES CXX)

add_executable(signed_integer_overflow main.cpp)
target_compile_features(signed_integer_overflow PUBLIC cxx_std_11)

include(GNUInstallDirs)
install(TARGETS signed_integer_overflow
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
