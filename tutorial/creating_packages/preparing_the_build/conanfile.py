from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout
from conan.tools.scm import Git
from conan.tools.build import check_min_cppstd


class helloRecipe(ConanFile):
    name = "hello"
    version = "1.0"

    # Optional metadata
    license = "<Put the package license here>"
    author = "<Put your name here> <And your email here>"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "<Description of hello package here>"
    topics = ("<Put some tag here>", "<here>", "<and here>")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    options = {"shared": [True, False], 
               "fPIC": [True, False],
               "with_fmt": [True, False]}

    default_options = {"shared": False, 
                       "fPIC": True,
                       "with_fmt": True}

    generators = "CMakeDeps"

    def validate(self):
        if self.options.with_fmt:
            check_min_cppstd(self, "11")

    def source(self):
        git = Git(self)
        git.clone(url="https://github.com/conan-io/libhello.git", target=".")
        # Please, be aware that using the head of the branch instead of an immutable tag
        # or commit is not a good practice in general
        git.checkout("optional_fmt")

    def requirements(self):
        if self.options.with_fmt:
            self.requires("fmt/8.1.1")

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        if self.options.with_fmt:
            tc.variables["WITH_FMT"] = True
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["hello"]
