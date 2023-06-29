from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout
from conan.tools.files import copy


class SdlExampleRecipe(ConanFile):
    name = "sdl-example"
    version = "1.0"
    package_type = "application"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*", "conan-logo.png"

    # Required Generators to integrate with CMake build scripts
    generators = "CMakeDeps", "CMakeToolchain"

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("libwebp/1.3.0", override=True) # Version conflict: libtiff/4.4.0->libwebp/1.2.4
        self.requires("sdl/2.26.1") # 2.26.5 is a version conflict with sdl_image
        self.requires("sdl_image/2.0.5")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        copy(self, "conan-logo.png", self.export_sources_folder, self.package_folder)
        cmake = CMake(self)
        cmake.install()
        

    
