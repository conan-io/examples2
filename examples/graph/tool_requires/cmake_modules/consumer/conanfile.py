from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout

class Conan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    tool_requires = "myfunctions/1.0"

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()

        deps = CMakeDeps(self)
        # By default 'myfunctions-config.cmake' is not created for tool_requires
        # we need to explicitly activate it
        deps.build_context_activated = ["myfunctions"]
        # and we need to tell to automatically load 'myfunctions' modules
        deps.build_context_build_modules = ["myfunctions"]
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()