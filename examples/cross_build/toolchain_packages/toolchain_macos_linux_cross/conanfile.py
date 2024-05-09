import os
from conan import ConanFile
from conan.tools.files import get, copy, download
from conan.errors import ConanInvalidConfiguration
from conan.tools.scm import Version


class LinuxGnuMacosToolchain(ConanFile):
    name = "linux-gnu-macos-toolchain"
    version = "13.2"

    license = "GPL-3.0-only"
    homepage = "https://github.com/messense/homebrew-macos-cross-toolchains/"
    description = "Conan package for test cross-compilation from Macos to Linux."
    settings = "os", "arch"
    package_type = "application"

    def _get_arch(self, arch):
        if arch in self._archs_aarch64():
            return "aarch64"
        elif arch in self._archs_x86_64():
            return "x86_64"

    def _archs_x86_64(self):
        return ["x86_64"]
    
    def _archs_aarch64(self):
        return ["armv8", "armv8_32", "armv8.3", "arm64ec"]

    def _get_toolchain(self, target_arch):
        return f"{self._get_arch(target_arch)}-unknown-linux-gnu"
    
    def _get_toolchain_name(self, arch_macos, arch_linux):
        return f"{arch_linux}-unknown-linux-gnu-{arch_macos}-darwin.tar.gz"
    
    def _hash_validator(self, toolchain):
        d = {
            "aarch64-unknown-linux-gnu-aarch64-darwin.tar.gz": "a87669a9df908d8d8859849a0f9fc0fb287561a4e449c21dade10663d42d2ccb",
            "aarch64-unknown-linux-gnu-x86_64-darwin.tar.gz": "6979291e34064583ac8b12a8b6b99ec6829caf22f47bcb68b646365ec9e24690",
            "x86_64-unknown-linux-gnu-aarch64-darwin.tar.gz": "bb59598afd84b4d850c32031a4fa64c928fb41f8ece4401553b6c23714efbc47",
            "x86_64-unknown-linux-gnu-x86_64-darwin.tar.gz": "86e28c979e5ca6d0d1019c9b991283f2ab430f65cee4dc1e4bdf85170ff7c4f2"
        }
        return d[toolchain]

    def _get_toolchain_url(self):
        arch_macos = self._get_arch(self.settings.arch)
        arch_linux = self._get_arch(self.settings_target.arch)
        toolchain = self._get_toolchain_name(arch_macos, arch_linux)
        hash_sha256 = self._hash_validator(toolchain)
        base_url = f"https://github.com/messense/homebrew-macos-cross-toolchains/releases/download/v13.2.0/{toolchain}"
        return base_url, hash_sha256

    def package_id(self):
        self.info.settings_target = self.settings_target
        # We only want the ``arch`` setting
        self.info.settings_target.rm_safe("os")
        self.info.settings_target.rm_safe("compiler")
        self.info.settings_target.rm_safe("build_type")

    def validate(self):
        valid_archs = self._archs_aarch64() + self._archs_x86_64()
        if self.settings.os != "Macos" or self._get_arch(self.settings.arch) == None:
            raise ConanInvalidConfiguration(f"This toolchain only supports building from Macos-{','.join(valid_archs)}. "
                                           f"{self.settings.os}-{self.settings.arch} is not supported.")

        if self.settings_target.os != "Linux" or self._get_arch(self.settings_target.arch) == None:
            raise ConanInvalidConfiguration(f"This toolchain only supports building for Linux-{','.join(valid_archs)}. "
                                           f"{self.settings_target.os}-{self.settings_target.arch} is not supported.")

    def build(self):
        url, sha256 = self._get_toolchain_url()
        get(self, url, strip_root=False, sha256=sha256)            

    def package(self):
        toolchain = self._get_toolchain(self.settings_target.arch)
        dirs_to_copy = [toolchain, "bin", "include", "lib", "libexec"]
        for dir_name in dirs_to_copy:
            copy(self, pattern=f"{dir_name}/*", src=self.build_folder, dst=self.package_folder, keep_path=True)
        copy(self, "LICENSE", src=self.build_folder, dst=os.path.join(self.package_folder, "licenses"), keep_path=False)

    def package_info(self):
        toolchain = self._get_toolchain(self.settings_target.arch)
        self.cpp_info.bindirs.append(os.path.join(self.package_folder, toolchain, "bin"))

        self.conf_info.define("tools.build:compiler_executables", {
            "c":   f"{toolchain}-gcc",
            "cpp": f"{toolchain}-g++",
            "asm": f"{toolchain}-as"
        })
