import os
from conan import ConanFile
from conan.tools.files import load, copy
from conan.tools.cmake import CMake


class PkgSay(ConanFile):
    name = "say"
    version = "1.0"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain"

    def layout(self):
        # The root of the project is one level above
        self.folders.root = ".." 
        # The source of the project (the root CMakeLists.txt) is the source folder
        self.folders.source = "."  
        self.folders.build = "build"

    def export_sources(self):
        # The path of the CMakeLists.txt and sources we want to export are one level above
        folder = os.path.join(self.recipe_folder, "..")
        copy(self, "*.txt", folder, self.export_sources_folder)
        copy(self, "src/*.cpp", folder, self.export_sources_folder)
        copy(self, "include/*.h", folder, self.export_sources_folder)
    
    def source(self):
        # Check that we can see that the CMakeLists.txt is inside the source folder
        cmake_file = load(self, "CMakeLists.txt")

    def build(self):
        # Check that the build() method can also access the CMakeLists.txt in the source folder
        path = os.path.join(self.source_folder, "CMakeLists.txt")
        cmake_file = load(self, path)

        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
