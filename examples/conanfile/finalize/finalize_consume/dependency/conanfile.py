import os
from conan import ConanFile
from conan.tools.files import copy, save

class Dependency(ConanFile):
    name = "dependency"
    version = "1.0"

    def package(self):
        save(self, os.path.join(self.package_folder, "file1.txt"), "Hello World!")
        save(self, os.path.join(self.package_folder, "file2.txt"), "Hi world")

    def finalize(self):
        self.output.info(f"Running finalize method in {self.package_folder}")
        copy(self, "file1.txt", src=self.immutable_package_folder, dst=self.package_folder)

    def package_info(self):
        self.output.info(f"Running package_info method in {self.package_folder}")
