cmake_minimum_required(VERSION 3.15)
project(gdexample LANGUAGES CXX)

find_package(godot-cpp REQUIRED CONFIG)
find_package(flecs REQUIRED CONFIG)

add_library(gdexample SHARED
    src/register_types.cpp
    src/swarm.cpp
)
target_link_libraries(gdexample PRIVATE godot-cpp flecs::flecs_static)

# Output as demo/bin/libgdexample.<target>.<ext>, the path the
# demo/bin/gdexample.gdextension file points Godot to. The generator
# expression prevents multi-config generators from adding a Release/ subfolder
set_target_properties(gdexample PROPERTIES
    OUTPUT_NAME "gdexample.${GODOTCPP_TARGET}"
    PREFIX "lib"
    LIBRARY_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/demo/bin>"
    RUNTIME_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/demo/bin>"
)
