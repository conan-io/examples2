import os
from conan import ConanFile
from conan.tools.files import copy

class Conan(ConanFile):
    name = "myfunctions"
    version = "1.0"
    exports_sources = ["*.cmake"]

    def package(self):
        copy(self, "*.cmake", self.source_folder, self.package_folder)

    def package_info(self):
        self.cpp_info.set_property("cmake_build_modules", ["myfunction.cmake"])
