from conan import ConanFile
from conan.tools.build import can_run
from conan.tools.cmake import cmake_layout, CMake
import os


class AppNCursesVersionConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"
    package_type = "application"
    exports_sources = "CMakeLists.txt", "ncurses_version.c"

    def requirements(self):
        if self.settings.os in ["Linux", "Macos", "FreeBSD"]:
            self.requires("ncurses/system")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

        app_path = os.path.join(self.build_folder, "ncurses_version")
        self.output.info(f"The example application has been successfully built.\nPlease run the executable using: '{app_path}'")
