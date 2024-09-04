import getpass
import os

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, rm, save


class whoisconanRecipe(ConanFile):
    name = "whoisconan"
    version = "1.0"
    package_type = "application"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*"

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        save(self, os.path.join(self.package_folder, "licenses", "LICENSE"), "Example")
        cmake = CMake(self)
        cmake.install()

    def finalize(self):
        copy(self, "*", src=self.immutable_package_folder, dst=self.package_folder)
        save(self, os.path.join(self.package_folder, "bin", "whoami.txt"), getpass.getuser())
