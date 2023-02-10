from conan import ConanFile


class Intro(ConanFile):
    name = "intro"

    def requirements(self):
        self.requires("matrix/1.1")

