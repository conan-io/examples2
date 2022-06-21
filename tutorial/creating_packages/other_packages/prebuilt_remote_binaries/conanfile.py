import os
from conan.tools.files import download, copy
from conan import ConanFile


class HelloConan(ConanFile):
    name = "hello"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"

    def build(self):
        base_url = "https://raw.githubusercontent.com/lasote/examples2/feature/prebuilt_binaries/tutorial" \
                   "/creating_packages/other_packages/prebuilt_binaries/vendor_foo_library"

        _os = str(self.settings.os).lower()
        _arch = str(self.settings.arch).lower()
        libname = "libfoo.a" if _os != "windows" else "foo.lib"
        url = "{}/{}/{}/{}".format(base_url, _os, _arch, libname)
        download(self, url, libname)

        url = "{}/{}/{}/include/foo.h".format(base_url, _os, _arch)
        download(self, url, "foo.h")

    def package(self):
        copy(self, "*.h", self.build_folder, os.path.join(self.package_folder, "include"))
        copy(self, "*.lib", self.build_folder, os.path.join(self.package_folder, "lib"))
        copy(self, "*.a", self.build_folder, os.path.join(self.package_folder, "lib"))

    def package_info(self):
        self.cpp_info.libs = ["foo"]