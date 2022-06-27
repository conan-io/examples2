from conan import ConanFile
from conan.tools.build import cross_building


class secure_scannerTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        self.tool_requires(self.tested_reference_str)

    def test(self):
        if not cross_building(self):
            extension = ".exe" if self.settings_build.os == "Windows" else ""
            self.run("secure_scanner{} mypath".format(extension), env="conanrun")
