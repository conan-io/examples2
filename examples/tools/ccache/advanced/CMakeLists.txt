cmake_minimum_required(VERSION 3.15)
project(ccache_demo)

add_executable(ccache_demo src/main.cpp)

find_package(fmt REQUIRED)
find_package(cpr REQUIRED)
find_package(RapidJSON REQUIRED)

target_link_libraries(ccache_demo
  PRIVATE
    cpr::cpr
    fmt::fmt
    rapidjson
)

install(TARGETS ccache_demo DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )
