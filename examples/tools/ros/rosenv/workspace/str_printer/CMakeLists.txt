cmake_minimum_required(VERSION 3.15)
project(str_printer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)  # Retrieved with Conan C/C++ Package Manager

add_library(str_printer src/str_printer.cpp)

target_include_directories(str_printer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/str_printer>
  $<INSTALL_INTERFACE:include>)

target_compile_features(str_printer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(str_printer PUBLIC fmt::fmt)

ament_export_targets(str_printerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(fmt)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS str_printer
  EXPORT str_printerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
