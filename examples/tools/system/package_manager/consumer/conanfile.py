from conan import ConanFile
from conan.tools.build import can_run
from conan.tools.cmake import cmake_layout, CMake
import os


class AppNCursesVersionConan(ConanFile):
    name = "ncurses-version"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"
    package_type = "application"
    exports_sources = "CMakeLists.txt", "ncurses_version.cpp"

    def requirements(self):
        if self.settings.os == "Linux":
            self.requires("ncurses/system")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libdirs = []
        self.cpp_info.includedirs = []