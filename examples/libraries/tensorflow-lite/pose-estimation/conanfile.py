from conan import ConanFile
from conan.tools.cmake import cmake_layout


class PoseEstimationRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    def requirements(self):
        self.requires("tensorflow-lite/2.10.0")
        self.requires("opencv/4.5.5")
        self.requires("libwebp/1.3.0", override=True)
        self.requires("eigen/3.4.0", override=True)

    def layout(self):
        cmake_layout(self)
