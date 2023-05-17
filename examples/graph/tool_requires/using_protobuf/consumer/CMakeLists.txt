cmake_minimum_required(VERSION 3.15)
project(consumer CXX)

find_package(libfoo REQUIRED)

add_executable(consumer src/consumer.cpp src/main.cpp)

target_link_libraries(consumer PRIVATE libfoo::libfoo)

install(TARGETS consumer DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )
