from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class WasmExampleRecipe(ConanFile):
    name = "wasm-example"
    version = "1.0"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("zlib/1.3.1")
        self.requires("fmt/11.1.4")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)

        # HEAPxx values need to be exported explicitly since Emscripten 4.0.7  
        # https://github.com/emscripten-core/emscripten/blob/main/ChangeLog.md#407---041525
        tc.extra_exelinkflags.append(
            "-sEXPORTED_FUNCTIONS=['_malloc','_free'] \
            -sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','getValue','setValue','HEAPF32'] \
            -sALLOW_MEMORY_GROWTH=1 \
            -sNO_EXIT_RUNTIME=1 \
            --shell-file ${CMAKE_SOURCE_DIR}/shell.html"
        )
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
