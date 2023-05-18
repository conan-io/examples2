import os
from conan import ConanFile
from conan.tools.files import save

class Pkg(ConanFile):
    name = "gcc"
    version = "1.0"
    options = {"myoption": [1, 2]}

    def package(self):
        # This fake compiler will print something different based on the option
        echo = f"@echo off\necho MYGCC={self.options.myoption}!!"
        save(self, os.path.join(self.package_folder, "bin", f"mygcc{self.options.myoption}.bat"), echo)
        save(self, os.path.join(self.package_folder, "bin", f"mygcc{self.options.myoption}.sh"), echo)
        os.chmod(os.path.join(self.package_folder, "bin", f"mygcc{self.options.myoption}.sh"), 0o777)
    