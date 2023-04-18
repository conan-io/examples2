cmake_minimum_required(VERSION 3.15)
project(pose-estimation CXX)

find_package(tensorflowlite REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(pose-estimation src/main.cpp)

target_link_libraries(pose-estimation PRIVATE tensorflow::tensorflowlite opencv::opencv)
