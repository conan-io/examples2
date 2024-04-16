from io import StringIO
from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout
import os


class TestPackageConan(ConanFile):
    settings = "os", "arch", "compiler", "build_type"
    generators = "CMakeToolchain", "VirtualBuildEnv"

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
        elif arch in self._archs_mipsel():
            return "mipsel"

    def _archs_mipsel(self):
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

    def build_requirements(self):
        self.tool_requires(self.tested_reference_str)

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def test(self):
        toolchain = f"{self._get_arch_linux(self.settings.arch)}-unknown-linux-gnu"
        self.run(f"{toolchain}-gcc --version")
        test_file = os.path.join(self.cpp.build.bindirs[0], "test_package")
        stdout = StringIO()
        self.run(f"file {test_file}", stdout=stdout)
        archs_32 = self._archs_i686() + self._archs_arm() + self._archs_armv7()
        if self.settings.arch in archs_32:
            assert "ELF 32-bit" in stdout.getvalue()
        else:
            assert "ELF 64-bit" in stdout.getvalue()
