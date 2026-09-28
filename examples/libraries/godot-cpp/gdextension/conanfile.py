from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout


class GDExtensionExample(ConanFile):
    package_type = "shared-library"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps"

    def requirements(self):
        self.requires("godot-cpp/10.0.0")
        self.requires("flecs/4.1.6")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        # Godot picks the library to load by its build "target", so we name
        # the output after the target godot-cpp was built with
        tc.cache_variables["GODOTCPP_TARGET"] = str(self.dependencies["godot-cpp"].options.target)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
