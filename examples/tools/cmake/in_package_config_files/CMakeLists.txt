cmake_minimum_required(VERSION 3.15)
project(pkg CXX)

add_library(mylib src/pkg.cpp)  # Use a different name than the package, to make sure

set_target_properties(mylib PROPERTIES PUBLIC_HEADER "include/pkg.h")
target_include_directories(mylib PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

# Use non default mypkgConfig name
install(TARGETS mylib EXPORT mypkgConfig)
export(TARGETS mylib
    NAMESPACE mypkg::  # to simulate a different name and see it works
    FILE "${CMAKE_CURRENT_BINARY_DIR}/mypkgConfig.cmake"
)
install(EXPORT mypkgConfig
    DESTINATION "${CMAKE_INSTALL_PREFIX}/pkg/cmake"
    NAMESPACE mypkg::
)
