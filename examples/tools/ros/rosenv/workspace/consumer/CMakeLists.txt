cmake_minimum_required(VERSION 3.15)
project(consumer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(str_printer REQUIRED)

add_executable(main src/main.cpp)

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/consumer>
  $<INSTALL_INTERFACE:include>)

target_compile_features(main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(main PUBLIC str_printer::str_printer)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
