cmake_minimum_required(VERSION 3.15)
project(wasm_example CXX)

find_package(Eigen3 REQUIRED)
add_executable(wasm_example main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

install(TARGETS wasm_example DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

# Required for Emscripten + embind
set(CMAKE_EXECUTABLE_SUFFIX ".html")

# set_target_properties(wasm_example PROPERTIES
#     LINK_FLAGS "-sEXPORTED_FUNCTIONS=['_malloc','_free'] -sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','getValue','setValue'] -sENVIRONMENT=web -sALLOW_MEMORY_GROWTH=1 -sNO_EXIT_RUNTIME=1  --shell-file ${CMAKE_SOURCE_DIR}/shell.html"
# )

set_target_properties(wasm_example PROPERTIES
    LINK_FLAGS
        "-sEXPORTED_FUNCTIONS=['_malloc','_free'] \
        -sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','getValue','setValue'] \
        -sENVIRONMENT=web \
        -sALLOW_MEMORY_GROWTH=1 \
        -sNO_EXIT_RUNTIME=1 \
        --shell-file ${CMAKE_SOURCE_DIR}/shell.html"
)
