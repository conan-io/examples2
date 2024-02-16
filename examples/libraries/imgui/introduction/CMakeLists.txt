cmake_minimum_required(VERSION 3.15)
project(dear-imgui-conan CXX)

find_package(imgui REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glew REQUIRED)

add_executable( dear-imgui-conan
                main.cpp
                opengl_shader.cpp
                file_manager.cpp
                opengl_shader.h
                file_manager.h
                bindings/imgui_impl_glfw.cpp
                bindings/imgui_impl_glfw.h
                bindings/imgui_impl_opengl3.cpp
                bindings/imgui_impl_opengl3.h
                bindings/imgui_impl_opengl3_loader.h
                assets/simple-shader.vs
                assets/simple-shader.fs )

add_custom_command(TARGET dear-imgui-conan
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/assets/simple-shader.vs $<TARGET_FILE_DIR:dear-imgui-conan>
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/assets/simple-shader.fs $<TARGET_FILE_DIR:dear-imgui-conan>
)

target_compile_definitions(dear-imgui-conan PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)
target_link_libraries(dear-imgui-conan imgui::imgui GLEW::GLEW glfw)
