cmake_minimum_required(VERSION 3.15)
project(compressor C)

find_package(ZLIB REQUIRED)

if(WIN32)
    find_package(base64 REQUIRED)
endif()

add_executable(${PROJECT_NAME} src/main.c)

if(WIN32)
    target_link_libraries(${PROJECT_NAME} ZLIB::ZLIB base64::base64)
else()
    target_link_libraries(${PROJECT_NAME} ZLIB::ZLIB)
endif()