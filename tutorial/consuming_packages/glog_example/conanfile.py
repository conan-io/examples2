from conans import ConanFile
from conans import CMake
from conans import tools
#from conan import ConanFile
#from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout


class GtestExampleConan(ConanFile):
    name = "gtest_example"
    version = "0.0.1"

    # Optional metadata
    license = "<Put the package license here>"
    author = "<Put your name here> <And your email here>"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "<Description of ConanSwigExample here>"
    topics = ("<Put some tag here>", "<here>", "<and here>")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    requires = ("glog/0.6.0")
    tool_requires = ("cmake/3.23.2", "ninja/1.11.0", "ccache/4.6")
    build_policy = "missing"
    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*"
    generators = "cmake"
 
    def build(self):
        cmake = CMake(self, "Ninja")
        cmake.configure()
        #cmake.configure(source_folder=self.src)
        cmake.build()
        cmake.install()

    def package_info(self):
#        self.cpp_info.libs = ["conan_swig_example_lib"]
        self.cpp_info.bin = ["myapp"]
