from io import StringIO
from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout
import os


class TestPackageConan(ConanFile):
    settings = "os", "arch", "compiler", "build_type"
    generators = "CMakeToolchain"

    def _get_arch(self, arch):
        if arch in self._archs_aarch64():
            return "aarch64"
        elif arch in self._archs_x86_64():
            return "x86_64"

    def _archs_x86_64(self):
        return ["x86_64"]

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
        toolchain = f"{self._get_arch(self.settings.arch)}-unknown-linux-gnu"
        self.run(f"{toolchain}-gcc --version")
        test_file = os.path.join(self.cpp.build.bindirs[0], "test_package")
        stdout = StringIO()
        self.run(f"file {test_file}", stdout=stdout)
        assert "ELF 64-bit" in stdout.getvalue()
