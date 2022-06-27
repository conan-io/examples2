from conan import ConanFile


class secure_scannerTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        self.tool_requires(self.tested_reference_str)

    def test(self):
        extension = ".exe" if self.settings_build.os == "Windows" else ""
        self.run("secure_scanner{} mypath".format(extension))
