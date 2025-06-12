from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps


class WasmAllocatorRecipe(ConanFile):
    name = "wasm-alloc"
    version = "1.0"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"
    
    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("icu/74.2")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
