cmake_minimum_required(VERSION 3.15)
project(pkg CXX)

add_library(pkg src/pkg.cpp)

set_target_properties(pkg PROPERTIES PUBLIC_HEADER "include/pkg.h")
target_include_directories(pkg PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

install(TARGETS pkg EXPORT pkgConfig)
export(TARGETS pkg
    NAMESPACE pkg::
    FILE "${CMAKE_CURRENT_BINARY_DIR}/pkgConfig.cmake"
)
install(EXPORT pkgConfig
    DESTINATION "${CMAKE_INSTALL_PREFIX}/pkg/cmake"
    NAMESPACE pkg::
)
