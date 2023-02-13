import os

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake


class SayConan(ConanFile):
    name = "say"
    version = "1.0"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*", "include/*"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):

        ## define project folder structure

        self.folders.source = "."
        self.folders.build = os.path.join("build", str(self.settings.build_type))
        self.folders.generators = os.path.join(self.folders.build, "generators")

        ## cpp.package information is for consumers to find the package contents in the Conan cache

        self.cpp.package.libs = ["say"]
        self.cpp.package.includedirs = ["include"] # includedirs is already set to 'include' by
                                                    # default, but declared for completion
        self.cpp.package.libdirs = ["lib"]         # libdirs is already set to 'lib' by
                                                    # default, but declared for completion

        ## cpp.source and cpp.build information is specifically designed for editable packages:

        # this information is relative to the source folder that is '.'
        self.cpp.source.includedirs = ["include"] # maps to ./include

        # this information is relative to the build folder that is './build/<build_type>', so it will 
        self.cpp.build.libdirs = ["."]  # map to ./build/<build_type> for libdirs

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
