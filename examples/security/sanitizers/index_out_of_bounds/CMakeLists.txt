cmake_minimum_required(VERSION 3.15)
project(index_out_of_bounds LANGUAGES CXX)

add_executable(index_out_of_bounds main.cpp)
target_compile_features(index_out_of_bounds PUBLIC cxx_std_11)

include(GNUInstallDirs)
install(TARGETS index_out_of_bounds
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
