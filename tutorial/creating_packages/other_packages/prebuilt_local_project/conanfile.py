import os
from conan import ConanFile
from conan.tools.files import copy
from conan.tools.cmake import cmake_layout


class helloRecipe(ConanFile):
    name = "hello"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def layout(self):
        cmake_layout(self)

    def package(self):
        local_include_folder = os.path.join(self.source_folder, self.cpp.source.includedirs[0])
        local_lib_folder = os.path.join(self.build_folder, self.cpp.build.libdirs[0])
        copy(self, "*.h", local_include_folder, os.path.join(self.package_folder, "include"), keep_path=False)
        copy(self, "*.lib", local_lib_folder, os.path.join(self.package_folder, "lib"), keep_path=False)
        copy(self, "*.a", local_lib_folder, os.path.join(self.package_folder, "lib"), keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ["hello"]
