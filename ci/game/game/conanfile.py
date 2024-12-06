from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout


class gameRecipe(ConanFile):
    name = "game"
    version = "1.0"
    package_type = "application"

    requires = "engine/[>=1.0 <2]"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*"

    def generate(self):
        tc = CMakeToolchain(self)
        tc.preprocessor_definitions["PKG_VERSION"] = f'"{self.version}"'
        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
