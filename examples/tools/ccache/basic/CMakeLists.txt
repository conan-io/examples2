cmake_minimum_required(VERSION 3.15)

project(basic CXX)

find_program(CCACHE_PROGRAM NAMES ccache)
if(CCACHE_PROGRAM)
    message(STATUS "ccache found: ${CCACHE_PROGRAM}, enabling compiler launcher.")
    # Set the compiler launcher property on the targets
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM} CACHE FILEPATH "CXX compiler cache used")
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM} CACHE FILEPATH "C compiler cache used")
else()
    message(WARNING "ccache not found. Compiler cache disabled.")
endif()


add_executable(basic src/basic.cpp src/main.cpp)

install(TARGETS basic DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )
