cmake_minimum_required(VERSION 3.8)
project(navigation_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

# Conan dependencies
find_package(yaml-cpp REQUIRED)

add_executable(navigator src/navigator.cpp)

target_compile_features(navigator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(navigator rclcpp rclcpp_action nav2_msgs yaml-cpp)

install(TARGETS navigator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
