import os
from conan import ConanFile
from conan.tools.build import cross_building


class secure_scannerTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        self.requires(self.tested_reference_str)

    def test(self):
        if not cross_building(self):
            self.run("secure_scanner mypath", env="conanrun")
