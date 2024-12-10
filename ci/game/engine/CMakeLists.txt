cmake_minimum_required(VERSION 3.15)
project(engine CXX)

find_package(ai CONFIG REQUIRED)
find_package(graphics CONFIG REQUIRED)

add_library(engine src/engine.cpp)
target_include_directories(engine PUBLIC include)
target_link_libraries(engine graphics::graphics ai::ai)

set_target_properties(engine PROPERTIES PUBLIC_HEADER "include/engine.h")
install(TARGETS engine)
