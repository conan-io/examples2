from conan import ConanFile

class Pkg(ConanFile):
    name = "game1"
    version = "1.0"

    def requirements(self):  
        self.requires("engine/1.0")
        self.requires("math/2.0", force=True)
