from conan import ConanFile
from conan.tools.cmake import cmake_layout, CMakeToolchain

class ConanApplication(ConanFile):
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("raylib/5.5")
