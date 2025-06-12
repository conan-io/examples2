from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.build import check_min_cppstd

class ConanApplication(ConanFile):
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps"

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("raylib/5.5")

    def validate(self):
        check_min_cppstd(self, "17")

    def generate(self):
        tc = CMakeToolchain(self)
        if self.settings.os == "Emscripten":
            tc.extra_exelinkflags.append(
                "-sUSE_GLFW=3 -sASYNCIFY --shell-file=${CMAKE_SOURCE_DIR}/shell.html"
            )
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
