
from os.path import join
from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout
from conan.tools.files import copy

class GreetingsConan(ConanFile):
    name = "greetings"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"
    exports_sources = "src/*"

    def build(self):
       cmake = CMake(self)
       cmake.configure()
       cmake.build()

    def layout(self):
        cmake_layout(self, src_folder="src")
        self.cpp.source.components["hello"].includedirs = ["."]
        self.cpp.source.components["bye"].includedirs = ["."]
        bt = "." if self.settings.os != "Windows" else str(self.settings.build_type)
        self.cpp.build.components["hello"].libdirs = [bt]
        self.cpp.build.components["bye"].libdirs = [bt]

    def package(self):
       copy(self, "*.h", src=self.source_folder,
                         dst=join(self.package_folder, "include"))
       copy(self, "*.lib", src=self.build_folder,
                           dst=join(self.package_folder, "lib"), keep_path=False)
       copy(self, "*.a", src=self.build_folder,
                         dst=join(self.package_folder, "lib"), keep_path=False)

    def package_info(self):
       self.cpp_info.components["hello"].libs = ["hello"]
       self.cpp_info.components["bye"].libs = ["bye"]

       self.cpp_info.set_property("cmake_file_name", "MYG")
       self.cpp_info.set_property("cmake_target_name", "MyGreetings::MyGreetings")
       self.cpp_info.components["hello"].set_property("cmake_target_name", "MyGreetings::MyHello")
       self.cpp_info.components["bye"].set_property("cmake_target_name", "MyGreetings::MyBye")
