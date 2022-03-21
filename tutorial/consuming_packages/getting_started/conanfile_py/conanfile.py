from conan import ConanFile
from conan.tools.cmake import cmake_layout
from conan.errors import ConanInvalidConfiguration


class CompressorRecipe(ConanFile):
    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def validate(self):
        if self.settings.os == "Macos" and self.settings.arch == "armv8":
            raise ConanInvalidConfiguration("ARM v8 not supported")

    def requirements(self):
        self.requires("zlib/1.2.11")
        # Use the system's CMake for Windows
        # and add base64 dependency
        if self.settings.os == "Windows":
            self.requires("base64/0.4.0")
        else:
            self.tool_requires("cmake/3.19.8")

    # do not introduce layout method yet
    # use conan install . --output-folder cmake-build-release
    # then explain you can use layout to pre-define that
    # def layout(self):
    #     cmake_layout(self)
