import os
from conan.tools.files import get, copy
from conan import ConanFile


class HelloConan(ConanFile):
    name = "hello"
    version = "0.1"
    settings = "os", "arch"

    def build(self):
        base_url = "https://github.com/conan-io/libhello/releases/download/0.0.1/"

        _os = {"Windows": "win", "Linux": "linux", "Macos": "macos"}.get(str(self.settings.os))
        _arch = str(self.settings.arch).lower()
        url = "{}/{}_{}.tgz".format(base_url, _os, _arch)
        get(self, url)

    def package(self):
        copy(self, "*.h", self.build_folder, os.path.join(self.package_folder, "include"))
        copy(self, "*.lib", self.build_folder, os.path.join(self.package_folder, "lib"))
        copy(self, "*.a", self.build_folder, os.path.join(self.package_folder, "lib"))

    def package_info(self):
        self.cpp_info.libs = ["hello"]
