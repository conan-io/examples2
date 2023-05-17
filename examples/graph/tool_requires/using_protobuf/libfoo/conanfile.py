from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps


class LibfooRecipe(ConanFile):
    name = "libfoo"
    version = "1.0"
    package_type = "library"

    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    generators = "CMakeDeps", "CMakeToolchain"
    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*", "include/*", "addressbook.proto"

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def requirements(self):
        self.requires("protobuf/3.18.1", transitive_headers=True)

    def build_requirements(self):
        # We want to use the same version as host, even if it's overridden!
        self.tool_requires("protobuf/<host_version>")

    def layout(self):
        cmake_layout(self)

    def build(self):
        protobuf_host_version = self.dependencies.host["protobuf"].ref.version
        protobuf_build_version = self.dependencies.build["protobuf"].ref.version
        # Checking that HOST/BUILD versions of protobuf are exactly the same
        self.output.warning(f"Protobuf HOST/BUILD versions: {protobuf_host_version}/{protobuf_build_version}")
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["libfoo"]
        self.cpp_info.requires = ["protobuf::libprotobuf"]

