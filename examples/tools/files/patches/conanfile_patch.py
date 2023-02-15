import os
from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout
from conan.tools.files import get, replace_in_file, patch


class helloRecipe(ConanFile):
    name = "hello"
    version = "1.0"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    exports_sources = "*.patch"

    def source(self):
        # Please, be aware that using the head of the branch instead of an immutable tag
        # or commit is not a good practice in general as the branch may change the contents
        get(self, "https://github.com/conan-io/libhello/archive/refs/heads/main.zip", strip_root=True)
        patch_file = os.path.join(self.export_sources_folder, "hello_patched.patch")
        patch(self, patch_file=patch_file)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["hello"]
