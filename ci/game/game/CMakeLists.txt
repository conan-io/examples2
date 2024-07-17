cmake_minimum_required(VERSION 3.15)
project(game CXX)

find_package(engine CONFIG REQUIRED)
add_executable(game src/game.cpp src/main.cpp)
target_link_libraries(game engine::engine)

install(TARGETS game DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )
