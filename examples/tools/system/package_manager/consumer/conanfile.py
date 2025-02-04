from conan import ConanFile
from conan.tools.build import can_run
from conan.tools.cmake import cmake_layout, CMake
import os


class AppNCursesVersionConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"
    package_type = "application"
    exports_sources = "CMakeLists.txt", "ncurses_version.c"

    @property
    def _supported_os(self):
        return ["Linux", "Macos", "FreeBSD"]

    def requirements(self):
        if self.settings.os in _supported_os:
            self.requires("ncurses/system")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

        if self.settings.os in self._supported_os:
            self.run(os.path.join(self.build_folder, "ncurses_version"), env="conanrun")
