from conan import ConanFile


class Engine(ConanFile):
    name = "engine"
    
    def requirements(self):
        self.requires("matrix/1.0")
