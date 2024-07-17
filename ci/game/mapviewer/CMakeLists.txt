cmake_minimum_required(VERSION 3.15)
project(mapviewer CXX)

find_package(graphics CONFIG REQUIRED)
add_executable(mapviewer src/mapviewer.cpp src/main.cpp)
target_link_libraries(mapviewer graphics::graphics)

install(TARGETS mapviewer DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )
