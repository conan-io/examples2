import os


from conan import ConanFile
from conan.tools.files import get, copy, download
from conan.errors import ConanInvalidConfiguration
from conan.tools.scm import Version



class ArmToolchainPackage(ConanFile):
    name = "arm-toolchain"
    version = "13.2"

    license = "GPL-3.0-only"

    homepage = "https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads"

    description = "Conan package for arm toolchain for Linux basic example"

    settings = "os", "arch", "compiler"
    package_type = "application"

    def package_id(self):
        del self.info.settings.compiler

    def validate(self):
        if self.settings_target:
            if self.settings_target.os != "Linux" or self.settings_target.arch not in ["armv7hf", "armv8"]:
                raise ConanInvalidConfiguration("This toolchain only supports building for Linux-armv7hf/armv8.",
                                            "{self.settings_target.os}-{self.settings_target.arch} not supported")

            if self.settings_target.compiler != "gcc":
                raise ConanInvalidConfiguration(f"The compiler is set to: '{self.settings.compiler}' but this toolchain only supports building with gcc.")

            if Version(self.settings_target.compiler.version) >= Version("14") or Version(self.settings_target.compiler.version) < Version("13"):
                raise ConanInvalidConfiguration(f"Invalid gcc version: '{self.version}'. Only 13.X versions supported for compiler.")

        if self.settings.arch != "x86_64" or self.settings.os != "Linux":
            raise ConanInvalidConfiguration("This toolchain is not compatible with: {self.settings.os}-{self.settings.arch}, can only run in Linux-x86_64.")


    def source(self):
        download(self, "https://developer.arm.com/GetEula?Id=37988a7c-c40e-4b78-9fd1-62c20b507aa8", "LICENSE", verify=False)
        get(self, "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-linux-gnueabihf.tar.xz",
            sha256 = "df0f4927a67d1fd366ff81e40bd8c385a9324fbdde60437a512d106215f257b3", strip_root=True)            

    def package(self):
        copy(self, pattern="arm-none-linux-gnueabihf/*", src=self.build_folder, dst=self.package_folder, keep_path=True)
        copy(self, pattern="bin/*", src=self.build_folder, dst=self.package_folder, keep_path=True)
        copy(self, pattern="include/*", src=self.build_folder, dst=self.package_folder, keep_path=True)
        copy(self, pattern="lib/*", src=self.build_folder, dst=self.package_folder, keep_path=True)
        copy(self, pattern="libexec/*", src=self.build_folder, dst=self.package_folder, keep_path=True)
        copy(self, pattern="LICENSE", src=self.build_folder, dst=os.path.join(self.package_folder, "licenses/"), keep_path=True)

    def package_info(self):
        self.cpp_info.bindirs.append(os.path.join(self.package_folder, "arm-none-linux-gnueabihf", "bin"))
        self.cpp_info.includedirs.append(os.path.join(self.package_folder, "arm-none-linux-gnueabihf", "include"))
        self.cpp_info.libdirs.append(os.path.join(self.package_folder, "arm-none-linux-gnueabihf", "lib"))

        #self.conf_info.append("tools.build:cflags", ["--specs=rdimon.specs"])
        #self.conf_info.append("tools.build:sharedlinkflags", ["--specs=rdimon.specs"])
        
        self.conf_info.define("tools.build:compiler_executables", {
            "c": "arm-none-linux-gnueabihf-gcc",
            "cpp": "arm-none-linux-gnueabihf-g++",
            "asm": "arm-none-linux-gnueabihf-as"
        })
