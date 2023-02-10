from conan import ConanFile


class Matrix(ConanFile):
    name = "matrix"
    version = "1.0"
    options = {"shared": [True, False]}
    default_options = {"shared": False}

    def package_info(self):
        self.output.info(f"I am a {self.package_type} library!!!")
