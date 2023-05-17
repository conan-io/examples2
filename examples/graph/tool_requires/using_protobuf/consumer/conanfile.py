from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout


class ConsumerRecipe(ConanFile):
    name = "consumer"
    version = "1.0"
    package_type = "application"
    settings = "os", "compiler", "arch", "build_type"
    generators = "CMakeDeps", "CMakeToolchain"
    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*"

    def requirements(self):
        self.requires("libfoo/1.0")
        # Overrides only the host requirement!
        self.requires("protobuf/3.21.9", override=True)

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
