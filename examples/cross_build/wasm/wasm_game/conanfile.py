from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout

class ConanApplication(ConanFile):
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("raylib/5.5")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
