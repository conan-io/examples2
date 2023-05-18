import os
from conan import ConanFile
from conan.tools.files import save


class Pkg(ConanFile):
    name = "gcc"
    
    def package(self):
        echo = f"@echo off\necho MYGCC={self.version}!!"
        save(self, os.path.join(self.package_folder, "bin", f"mygcc{self.version}.bat"), echo)
        save(self, os.path.join(self.package_folder, "bin", f"mygcc{self.version}.sh"), echo)
        os.chmod(os.path.join(self.package_folder, "bin", f"mygcc{self.version}.sh"), 0o777)