from conan import ConanFile
from conan.tools.cmake import cmake_layout


class compressorRecipe(ConanFile):
    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        self.requires("zlib/1.2.11")

    # do not introduce layout method yet
    # def layout(self):
    #     cmake_layout(self)
