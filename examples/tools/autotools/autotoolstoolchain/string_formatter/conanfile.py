from conan import ConanFile
from conan.tools.gnu import AutotoolsToolchain, PkgConfigDeps, Autotools
from conan.tools.env import VirtualBuildEnv, Environment
from conan.tools.microsoft import unix_path, is_msvc
import os


class StringFormatterConanFile(ConanFile):
    settings = "os", "arch", "compiler", "build_type"

    def requirements(self):
        self.requires("fmt/9.1.0")

    def build_requirements(self):
        if self.settings.os == "Windows":
            if not self.conf.get("tools.microsoft.bash:path", check_type=str):
                self.tool_requires("msys2/cci.latest")
            self.tool_requires("autoconf/2.71")
            self.tool_requires("automake/1.16.5")
            self.tool_requires("pkgconf/1.9.3")

    def generate(self):
        if self.settings.os == "Windows":
            env = VirtualBuildEnv(self)
            env.generate()
            if is_msvc(self):
                env = Environment()
                env.define("CC", "cl -nologo")
                env.define("CXX", "cl -nologo")
                env.vars(self, scope="build").save_script("compiler_conf")
        tc = AutotoolsToolchain(self)
        tc.generate()
        tc = PkgConfigDeps(self)
        tc.generate()
