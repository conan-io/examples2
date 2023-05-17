import os, platform
from conan import ConanFile
from conan.tools.files import save, chdir

class Pkg(ConanFile):
    name = "wine"
    version = "1.0"
    
    def build_requirements(self):
        self.tool_requires("gcc/1.0", run=False, options={"myoption": 1})
        self.tool_requires("gcc/1.0", run=False, options={"myoption": 2})

    def generate(self):
        gcc1 = self.dependencies.build.get("gcc", options={"myoption": 1})
        assert gcc1.options.myoption == "1"
        gcc2 = self.dependencies.build.get("gcc", options={"myoption": 2})
        assert gcc2.options.myoption == "2"

    def build(self):
        ext = "bat" if platform.system() == "Windows" else "sh"
        self.run(f"mygcc1.{ext}")
        self.run(f"mygcc2.{ext}")