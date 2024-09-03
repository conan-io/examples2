import os

from conan import ConanFile


class Consumer(ConanFile):
    name = "consumer"
    version = "1.0"
    requires = "dependency/1.0"

    def generate(self):
        self.output.info("Running generate method")

        dep_package_folder = self.dependencies["dependency"].package_folder
        content_in_dep_package_folder = os.listdir(dep_package_folder)
        immutable_package_folder = self.dependencies["dependency"].immutable_package_folder
        content_in_immutable_package_folder = os.listdir(immutable_package_folder)

        self.output.info(f"Dependency package_folder: {dep_package_folder}")
        self.output.info(f"Content in dependency package_folder:\n{content_in_dep_package_folder}")
        self.output.info(f"Dependency immutable_package_folder: {immutable_package_folder}")
        self.output.info(f"Content in dependency immutable_package_folder:\n{content_in_immutable_package_folder}")
