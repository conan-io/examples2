cmake_minimum_required(VERSION 3.8)
project(consumer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message("[consumer] AMENT_PREFIX_PATH env: $ENV{AMENT_PREFIX_PATH}")
message("[consumer] COLCON_PREFIX_PATH env: $ENV{COLCON_PREFIX_PATH}")
message("[consumer] COLCON_CURRENT_PREFIX env: $ENV{COLCON_CURRENT_PREFIX}")
message("[consumer] CMAKE_PREFIX_PATH env: $ENV{CMAKE_PREFIX_PATH}")
message("[consumer] CMAKE_TOOLCHAIN_FILE env: $ENV{CMAKE_TOOLCHAIN_FILE}")
message("[consumer] CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(box2d REQUIRED)
find_package(Boost REQUIRED)
# find_package(Boost REQUIRED NO_DEFAULT_PATH)  # Does NOT find any Boost

# WITH CMAKE_TOOLCHAIN_FILE
# find_package(Boost REQUIRED)  # Finds system Boost if CMAKE_PREFIX_PATH not set
# Commands:
# $ conan install consumer/conanfile.txt --output-folder install -g CMakeDeps -g CMakeToolchain
# $ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/ros2-examples/conan_install_example/install/conan_toolchain.cmake

# WITHOUT ANY ADDITIONAL PARAMETER
# find_package(Boost 1.84.0 REQUIRED)  # WORKS
# find_package(Boost REQUIRED HINTS "${CMAKE_INSTALL_PREFIX}")  # WORKS
# Commands:
# $ conan install consumer/conanfile.txt --output-folder install
# $ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

message("Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message("Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
message("Boost_LIBRARIES: ${Boost_LIBRARIES}")
message("Boost_VERSION: ${Boost_VERSION}")

add_library(consumer src/hello.cpp)

target_include_directories(consumer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/consumer>
  $<INSTALL_INTERFACE:include>)

target_compile_features(consumer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(consumer box2d Boost)

ament_export_targets(consumerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(box2d Boost)

install(
  DIRECTORY include/
  DESTINATION include
)

#ament_environment_hooks("${CMAKE_INSTALL_PREFIX}/conanrun.sh")

install(
  TARGETS consumer
  EXPORT consumerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
