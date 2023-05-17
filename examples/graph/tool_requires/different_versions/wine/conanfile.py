import os, platform
from conan import ConanFile
from conan.tools.files import save, chdir


class Pkg(ConanFile):
    name = "wine"
    version = "1.0"

    def build_requirements(self):
        # If we specify "run=False" they no longer conflict
        self.tool_requires("gcc/1.0", run=False)
        self.tool_requires("gcc/2.0", run=False)

    def generate(self):
        # It is possible to individually reference each one
        gcc1 = self.dependencies.build["gcc/1.0"]
        assert gcc1.ref.version == "1.0"
        gcc2 = self.dependencies.build["gcc/2.0"]
        assert gcc2.ref.version == "2.0"

    def build(self):
        ext = "bat" if platform.system() == "Windows" else "sh"
        self.run(f"mygcc1.0.{ext}")
        self.run(f"mygcc2.0.{ext}")
