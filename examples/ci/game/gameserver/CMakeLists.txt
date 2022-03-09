cmake_minimum_required(VERSION 3.15)
project(gameserver CXX)

find_package(physx CONFIG REQUIRED)
add_executable(gameserver src/gameserver.cpp src/main.cpp)
target_link_libraries(gameserver physx::physx)

install(TARGETS gameserver DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )
