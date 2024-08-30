import os

from conan import ConanFile


class Consumer(ConanFile):
    name = "consumer"
    version = "1.0"
    requires = "dependency/1.0"

    def generate(self):
        self.output.info("Running generate method")
        self.output.info(
            f"Dependency package_folder: {self.dependencies["dependency"].package_folder}"
        )
        self.output.info(
            f"Content in dependency package_folder:\n{os.listdir(self.dependencies["dependency"].package_folder)}"
        )
        self.output.info(
            f"Dependency immutable_package_folder: {self.dependencies["dependency"].immutable_package_folder}"
        )
        self.output.info(
            f"Content in dependency immutable_package_folder:\n{os.listdir(self.dependencies["dependency"].immutable_package_folder)}"
        )
