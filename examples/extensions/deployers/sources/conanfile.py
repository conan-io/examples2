from conan import ConanFile


class BasicConanfile(ConanFile):
    name = "pkg"
    version = "1.0"
    description = "A basic recipe"

    # The requirements method allows you to define the dependencies of your recipe
    def requirements(self):
        # Each call to self.requires() will add the corresponding requirement
        # to the current list of requirements
        self.requires("zlib/1.2.13")
        self.requires("mcap/0.5.0")
