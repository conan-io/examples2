import os

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy


class WasmExampleRecipe(ConanFile):
    name = "wasm-example"
    version = "1.0"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("eigen/3.4.0")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)

        tc.extra_exelinkflags.append(
            "-sEXPORTED_FUNCTIONS=['_malloc','_free'] \
            -sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','getValue','setValue'] \
            -sENVIRONMENT=web \
            -sALLOW_MEMORY_GROWTH=1 \
            -sNO_EXIT_RUNTIME=1 \
            --shell-file ${CMAKE_SOURCE_DIR}/shell.html"
        )

        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
