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
    description = "Conan package for the ARM toolchain, targeting different Linux ARM architectures."
    settings = "os", "arch"
    package_type = "application"

    def _get_arch_linux(self, arch):
        if arch in self._archs_aarch64():
            return "aarch64"
        elif arch in self._archs_armv7():
            return "armv7"
        elif arch in self._archs_arm():
            return "arm"
        elif arch in self._archs_x86_64():
            return "x86_64"
        elif arch in self._archs_i686():
            return "i686"
        elif arch in self._arch_mipsel():
            return "mipsel"

    def _get_arch_macos(self, arch):
        if arch in self._archs_aarch64():
            return "aarch64"
        elif arch in self._archs_x86_64():
            return "x86_64"

    def _arch_mipsel(self):
        return ["mips", "mips64"]

    def _archs_i686(self):
        return ["x86"]

    def _archs_x86_64(self):
        return ["x86_64"]

    def _archs_arm(self):
        return [ "armv4", "armv4i", "armv5el", "armv5hf", "armv6",]

    def _archs_armv7(self):
        return ["armv7", "armv7hf", "armv7s", "armv7k"]
    
    def _archs_aarch64(self):
        return ["armv8", "armv8_32", "armv8.3", "arm64ec"]

    def _valid_platforms(self):
        return ["gnu", "musl", "gnueabi", "gnueabihf"]

    def _get_toolchain(self, target_arch):
        return f"{self._get_arch_linux(target_arch)}-unknown-linux-gnu"
    
    def _hash_validator(self, toolchain):
        d = {}
        d["aarch64-unknown-linux-gnu-aarch64-darwin.tar.gz"] = "a87669a9df908d8d8859849a0f9fc0fb287561a4e449c21dade10663d42d2ccb"
        d["aarch64-unknown-linux-gnu-x86_64-darwin.tar.gz"] = "6979291e34064583ac8b12a8b6b99ec6829caf22f47bcb68b646365ec9e24690"
        d["aarch64-unknown-linux-musl-aarch64-darwin.tar.gz"] = "3f60dbda3b2934857cc63b27e1e680e36b181f3df9bbae9ec207989f47b0e7aa"
        d["aarch64-unknown-linux-musl-x86_64-darwin.tar.gz"] = "15a7166de1b364e591d6b0206d127b67d15e88555f314170088f5e9ccf0ab068"
        d["arm-unknown-linux-gnueabi-aarch64-darwin.tar.gz"] = "ee6265fab91e120afe4bb48fc86984c8edb500fd02f456275f9a445a9051e1fa"
        d["arm-unknown-linux-gnueabi-x86_64-darwin.tar.gz"] = "88a49bcdb1addcc9ffee6f844dc44bca412b8156c47f6f5cd0ab2d27a19f50dd"
        d["arm-unknown-linux-gnueabihf-aarch64-darwin.tar.gz"] = "84fe0ca9f2eb80103565065ea672568267b4405e0ececc69cedd31682f3cece1"
        d["arm-unknown-linux-gnueabihf-x86_64-darwin.tar.gz"] = "d724a44b03c3f51469c938170820c597ce8b625d3def17a29d1d50560c859f6c"
        d["arm-unknown-linux-musleabihf-aarch64-darwin.tar.gz"] = "5ab3b707f968e977e62332ec9cdb17689aa150688aafa65f36998ea5c1b5141c"
        d["arm-unknown-linux-musleabihf-x86_64-darwin.tar.gz"] = "cdb9928329aa0e3eb4ac5958e4192a31b2bc0611a7ad076028bac4ed48974680"
        d["armv7-unknown-linux-gnueabihf-aarch64-darwin.tar.gz"] = "3d6f308e408fc769e2c8c5a90eda74ad8c6f4f4c1c786d4419480a37bc8e2ed1"
        d["armv7-unknown-linux-gnueabihf-x86_64-darwin.tar.gz"] = "e1b4a0dafce3df7287b813a84d373e3edd7f122054f39accfbcdfaa4a31f9598"
        d["armv7-unknown-linux-musleabihf-aarch64-darwin.tar.gz"] = "f88a2d4c8cc1a7e26f9373b0f9ef7eda918690156cdd5c8a9b123a6a0b55199e"
        d["armv7-unknown-linux-musleabihf-x86_64-darwin.tar.gz"] = "ef2d1dadaf29fa2e00afe5fdd07a27e5fdd956a82173b0ce67998034e9a727b5"
        d["i686-unknown-linux-gnu-aarch64-darwin.tar.gz"] = "96f9285b10d81c8c2f5bbb3364b5fe1472082e6c74fdd18522d5d3b09a8c0128"
        d["i686-unknown-linux-gnu-x86_64-darwin.tar.gz"] = "ef232d85d05286bea61863ae47c48d5aa2c5adf8711d6c6fb85373150cbb335d"
        d["i686-unknown-linux-musl-aarch64-darwin.tar.gz"] = "774c58d86877cb8a9f21705418137f2564b0c7bdc60db3d9461781c12803917d"
        d["i686-unknown-linux-musl-x86_64-darwin.tar.gz"] = "0c56ec34be51295a3da5b9c890ebe77a209ca707ff163ec377ba3b3f11bd5703"
        d["mipsel-unknown-linux-gnu-aarch64-darwin.tar.gz"] = "b4e521bb7c28ed2b66f94f6a1bb6e840066fcbe1e4efde01528921cda3a07e99"
        d["mipsel-unknown-linux-gnu-x86_64-darwin.tar.gz"] = "5585d3890d5b978f67e39812203667b1ddf1719ff3e0b6ce06d9bdce8e7a0903"
        d["x86_64-unknown-linux-gnu-aarch64-darwin.tar.gz"] = "bb59598afd84b4d850c32031a4fa64c928fb41f8ece4401553b6c23714efbc47"
        d["x86_64-unknown-linux-gnu-x86_64-darwin.tar.gz"] = "86e28c979e5ca6d0d1019c9b991283f2ab430f65cee4dc1e4bdf85170ff7c4f2"
        d["x86_64-unknown-linux-musl-aarch64-darwin.tar.gz"] = "de0a12a677f3b91449e9c52a62f3d06c4c1a287aa26ba0bc36f86aaa57c24b55"
        d["x86_64-unknown-linux-musl-x86_64-darwin.tar.gz"] = "ff0f635766f765050dc918764c856247614c38e9c4ad27c30f85c0af4b21e919"
        return d[toolchain]

    def _get_toolchain_url(self, macos_arch, linux_arch):
        macos_name = self._get_arch_macos(macos_arch)
        linux_name = self._get_arch_linux(linux_arch)
        platform = "gnu"
        toolchain = f"{linux_name}-unknown-linux-{platform}-{macos_name}-darwin.tar.gz"
        hash_sha256 = self._hash_validator(toolchain)
        base_url = f"https://github.com/messense/homebrew-macos-cross-toolchains/releases/download/v13.2.0/{toolchain}"
        return base_url, hash_sha256

    def validate(self):
        if self.settings.os != "Macos" or self._get_arch_macos(self.settings.arch) == None:
            valid_archs_macos = self._archs_aarch64() + self._archs_x86_64()
            raise ConanInvalidConfiguration(f"This toolchain only supports building from Macos-{','.join(valid_archs_macos)}. "
                                           f"{self.settings.os}-{self.settings.arch} is not supported.")

        if self.settings_target.os != "Linux" or self._get_arch_linux(self.settings_target.arch) == None:
            valid_archs_linux = self._arch_mipsel() + self._archs_i686() + self._archs_x86_64() + self._archs_arm() + self._archs_armv7() + self._archs_aarch64()
            raise ConanInvalidConfiguration(f"This toolchain only supports building for Linux-{','.join(valid_archs_linux)}. "
                                           f"{self.settings_target.os}-{self.settings_target.arch} is not supported.")

    def build(self):
        url, sha256 = self._get_toolchain_url(self.settings.arch, self.settings_target.arch)
        get(self, url, strip_root=False, sha256=sha256)            

    def package(self):
        toolchain = self._get_toolchain(self.settings_target.arch)
        dirs_to_copy = [toolchain, "bin", "include", "lib", "libexec"]
        for dir_name in dirs_to_copy:
            copy(self, pattern=f"{dir_name}/*", src=self.build_folder, dst=self.package_folder, keep_path=True)
        copy(self, "LICENSE", src=self.build_folder, dst=os.path.join(self.package_folder, "licenses"), keep_path=False)

    def package_id(self):
        self.info.settings_target = self.settings_target
        # We only want the ``arch`` setting
        self.info.settings_target.rm_safe("os")
        self.info.settings_target.rm_safe("compiler")
        self.info.settings_target.rm_safe("build_type")

    def package_info(self):
        toolchain = self._get_toolchain(self.settings_target.arch)
        self.cpp_info.bindirs.append(os.path.join(self.package_folder, toolchain, "bin"))

        self.conf_info.define("tools.build:compiler_executables", {
            "c":   f"{toolchain}-gcc",
            "cpp": f"{toolchain}-g++",
            "asm": f"{toolchain}-as"
        })
