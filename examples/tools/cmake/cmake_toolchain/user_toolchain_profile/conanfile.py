from conan import ConanFile
from conan.tools.cmake import CMake

class AppConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    exports_sources = "CMakeLists.txt"
    name = "foo"
    version = "1.0"
    generators = "CMakeToolchain"

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
