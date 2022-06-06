from conan import ConanFile
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
        # Add base64 dependency only for Windows
        if self.settings.os == "Windows":
            self.requires("base64/0.4.0")
        if self.settings.os != "Windows":  # we need cmake 3.19 in other platforms
            self.tool_requires("cmake/3.19.8")

    def layout(self):
        self.folders.build = f"build/{str(self.settings.build_type)}"
        self.folders.generators = "build"
