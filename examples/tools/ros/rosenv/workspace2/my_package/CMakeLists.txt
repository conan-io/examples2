cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Conan dependencies
find_package(yaml-cpp REQUIRED)

add_executable(package src/main.cpp)

target_compile_features(package PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(package rclcpp rclcpp_action nav2_msgs yaml-cpp)

install(TARGETS package
  DESTINATION lib/${PROJECT_NAME})

ament_package()
