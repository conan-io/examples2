from conan import ConanFile


class Game(ConanFile):
    name = "game"
    version = "1.0"
    
    def requirements(self):
        self.requires("engine/1.0")
        self.requires("intro/1.0")
