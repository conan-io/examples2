from conan import ConanFile
from conan.tools.build import can_run
from conan.tools.files import copy


class SdlExampleTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        self.requires(self.tested_reference_str)

    def generate(self):
        copy(self, "*.png", self.dependencies[self.tested_reference_str].package_folder, self.build_folder)

    def test(self):
        if can_run(self):
            self.run("sdl-example", env="conanrun")
