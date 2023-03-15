from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout
from conan.tools.scm import Git
from conan.tools.files import load, update_conandata


class helloRecipe(ConanFile):
    name = "hello"
    version = "0.1"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    generators = "CMakeDeps", "CMakeToolchain"

    def export(self):
        git = Git(self, self.recipe_folder)
        scm_url, scm_commit = git.get_url_and_commit()
        self.output.info(f"Obtained URL: {scm_url} and {scm_commit}")
        # we store the current url and commit in conandata.yml
        update_conandata(self, {"sources": {"commit": scm_commit, "url": scm_url}})

    def source(self):
        # we recover the saved url and commit from conandata.yml and use them to get sources
        git = Git(self)
        sources = self.conan_data["sources"]
        self.output.info(f"Cloning sources from: {sources}")
        git.clone(url=sources["url"], target=".")
        git.checkout(commit=sources["commit"])

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["hello"]
