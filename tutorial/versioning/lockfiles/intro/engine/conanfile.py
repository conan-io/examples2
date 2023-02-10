from conan import ConanFile


class Engine(ConanFile):
    name = "engine"
    settings = "arch"

    def requirements(self):
        self.requires("matrix/[>=1.0 <2.0]")
        if self.settings.arch == "x86":
            self.requires("sound32/[>=1.0 <2.0]")
