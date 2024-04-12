import os
from conan import ConanFile
from conan.tools.files import get, copy, download
from conan.errors import ConanInvalidConfiguration
from conan.tools.scm import Version


class ArmToolchainPackage(ConanFile):
    name = "arm-toolchain-linux"
    version = "13.2"

    license = "GPL-3.0-only"
    # homepage = "https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads"
    description = "Conan package for the ARM toolchain, targeting different Linux ARM architectures."
    settings = "os", "arch"
    package_type = "application"

    def _archs32(self):
        return ["armv6", "armv7", "armv7hf"]
    
    def _archs64(self):
        return ["armv8", "armv8.3"]

    def _get_toolchain(self, target_arch):
        return "x86_64-unknown-linux-gnu"
        # if target_arch in self._archs32():
        #     return ("arm-none-linux-gnueabihf", 
        #             "df0f4927a67d1fd366ff81e40bd8c385a9324fbdde60437a512d106215f257b3")
        # else:
        #     return ("aarch64-none-linux-gnu", 
        #             "12fcdf13a7430655229b20438a49e8566e26551ba08759922cdaf4695b0d4e23")

    def validate(self):
        if self.settings.arch != "armv8" or self.settings.os != "Macos":
            raise ConanInvalidConfiguration(f"This toolchain is not compatible with {self.settings.os}-{self.settings.arch}. "
                                            "It can only run on Macos-armv8")

        # valid_archs = self._archs32() + self._archs64()
        # if self.settings_target.os != "Linux" or self.settings_target.arch not in valid_archs:
        #     raise ConanInvalidConfiguration(f"This toolchain only supports building for Linux-{valid_archs.join(',')}. "
        #                                    f"{self.settings_target.os}-{self.settings_target.arch} is not supported.")

        if self.settings_target.compiler != "gcc":
            raise ConanInvalidConfiguration(f"The compiler is set to '{self.settings_target.compiler}', but this "
                                            "toolchain only supports building with gcc.")

        if Version(self.settings_target.compiler.version) >= Version("14") or Version(self.settings_target.compiler.version) < Version("13"):
            raise ConanInvalidConfiguration(f"Invalid gcc version '{self.settings_target.compiler.version}'. "
                                            "Only 13.X versions are supported for the compiler.")

    def source(self):
        pass

    def build(self):
        get(self, "https://github.com/messense/homebrew-macos-cross-toolchains/releases/download/v13.2.0/x86_64-unknown-linux-gnu-aarch64-darwin.tar.gz") #, strip_root=True)            

    def package(self):
        # toolchain, _ = self._get_toolchain(self.settings_target.arch)
        dirs_to_copy = ["bin", "include", "lib", "libexec", "share", "x86_64-unknown-linux-gnu"]
        for dir_name in dirs_to_copy:
            copy(self, pattern=f"{dir_name}/*", src=self.build_folder, dst=self.package_folder, keep_path=True)
        # copy(self, "LICENSE", src=self.build_folder, dst=os.path.join(self.package_folder, "licenses"), keep_path=False)

    def package_id(self):
        self.info.settings_target = self.settings_target
        # We only want the ``arch`` setting
        self.info.settings_target.rm_safe("os")
        self.info.settings_target.rm_safe("compiler")
        self.info.settings_target.rm_safe("build_type")

    def package_info(self):
        toolchain = "x86_64-unknown-linux-gnu"
        self.cpp_info.bindirs.append(os.path.join(self.package_folder, toolchain, "bin"))

        self.conf_info.define("tools.build:compiler_executables", {
            "c":   f"{toolchain}-gcc",
            "cpp": f"{toolchain}-g++",
            "asm": f"{toolchain}-as"
        })
