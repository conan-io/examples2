from conan import ConanFile


class Engine(ConanFile):
    name = "engine"
    version = "1.0"
    # Not strictly necessary because this is already the matrix default
    default_options = {"matrix*:shared": False}

    def requirements(self):
        self.requires("matrix/1.0")
