from conan import ConanFile
from conan.errors import ConanInvalidConfiguration


class Intro(ConanFile):
    name = "intro"
    version = "1.0"
    default_options = {"matrix*:shared": True}

    def requirements(self):
        self.requires("matrix/1.0")

    def validate(self):
        if not self.dependencies["matrix"].options.shared:
            raise ConanInvalidConfiguration("Intro package doesn't work with static matrix library")