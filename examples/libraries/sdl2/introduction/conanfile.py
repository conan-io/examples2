from conan import ConanFile
from conan.tools.cmake import cmake_layout

class Sdl2Example(ConanFile):
    generators = "CMakeToolchain", "CMakeDeps"
    settings = "os", "build_type", "arch", "compiler" # This is because we are using the cmake layout!

    def requirements(self):
        self.requires("sdl_ttf/[~2.0]")
        self.requires("sdl_image/[~2.0]")
        # self.requires("sdl/[~2.26]")
        self.requires("libpng/1.6.40", override=True)
        self.requires("sdl/2.26.5", override=True)

    def build_requirements(self):
        self.tool_requires("cmake/[>=3.23]")
        
    def layout(self):
        cmake_layout(self)

