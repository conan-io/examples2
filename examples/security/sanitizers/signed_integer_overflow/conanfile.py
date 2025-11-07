from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout, CMakeToolchain

required_conan_version = ">=2.1.0"

class SignedIntegerOverflowConan(ConanFile):
    name = "signed_integer_overflow"
    version = "0.1.0"
    settings = "os", "arch", "compiler", "build_type"
    exports_sources = "CMakeLists.txt", "main.cpp"
    package_type = "application"
    languages = ["C++"]

    def layout(self):
        cmake_layout(self)

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