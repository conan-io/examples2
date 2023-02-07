import os

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout


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
        self.folders.source = "."
        build_type = str(self.settings.build_type)
        build_folder = os.path.join("build", build_type)
        self.folders.build = build_folder

        self.folders.generators = os.path.join(self.folders.build, "generators")

        self.cpp.package.libs = ["say"]
        self.cpp.package.includedirs = ["include"] # includedirs is already set to this value by
                                                    # default, but declared for completion

        # this information is relative to the source folder that is '.'
        self.cpp.source.includedirs = ["include"] # maps to ./include

        # this information is relative to the build folder that is './build/<build_type>', so it will 
        self.cpp.build.libdirs = ["."]  # map to ./build/<build_type> for libdirs
        self.cpp.build.bindirs = ["bin"]  # map to ./build/<build_type>/bin for bindirs

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        # we can also know where the executable we are building is
        self.run(os.path.join(self.build_folder, self.cpp.build.bindir, "hello"))

    def package(self):
        cmake = CMake(self)
        cmake.install()
