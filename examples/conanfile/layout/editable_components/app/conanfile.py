
import os
from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout

class GreetingsTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    def requirements(self):
        self.requires("greetings/0.1")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        self.run(os.path.join(self.cpp.build.bindirs[0], "example"))
        self.run(os.path.join(self.cpp.build.bindirs[0], "example2"))
