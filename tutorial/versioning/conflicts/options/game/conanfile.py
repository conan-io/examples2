from conan import ConanFile


class Game(ConanFile):
    name = "game"
    version = "1.0"
    default_options = {"matrix*:shared": True}
    
    def requirements(self):
        self.requires("engine/1.0")
        self.requires("intro/1.0")
