cmake_minimum_required(VERSION 3.15)
project(whoisconan CXX)

add_executable(whoisconan src/main.cpp)

install(TARGETS whoisconan DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )
