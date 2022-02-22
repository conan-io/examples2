from conan import ConanFile

class Pkg(ConanFile):
    name = "engine"
    version = "1.0"

    requires = "math/1.0"
