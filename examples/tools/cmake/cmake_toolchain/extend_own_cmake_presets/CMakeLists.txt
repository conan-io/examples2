cmake_minimum_required(VERSION 3.15)
project(foo CXX)



add_executable(foo src/foo.cpp src/main.cpp)



install(TARGETS foo DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )
