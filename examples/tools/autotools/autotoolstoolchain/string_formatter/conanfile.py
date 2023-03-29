from conan import ConanFile
from conan.tools.gnu import AutotoolsToolchain, PkgConfigDeps
from conan.tools.env import VirtualBuildEnv, Environment
from conan.tools.microsoft import unix_path
import os


class StringFormatterConanFile(ConanFile):
    settings = "os", "arch", "compiler", "build_type"

    @property
    def _default_cc(self):
        return {
            "gcc": "gcc",
            "clang": "clang",
            "Visual Studio": "cl -nologo",
            "msvc": "cl -nologo",
            "apple-clang": "clang",
        }

    def _system_compiler(self, cxx=False):
        system_cc = self._default_cc.get(str(self.settings.compiler))
        if system_cc and cxx:
            if self.settings.compiler == "gcc":
                system_cc = "g++"
            elif "clang" in self.settings.compiler:
                system_cc = "clang++"
        return system_cc

    def requirements(self):
        self.requires("fmt/9.1.0")

    def build_requirements(self):
        if self.settings.os == "Windows":
            if not self.conf.get("tools.microsoft.bash:path", check_type=str):
                self.tool_requires("msys2/cci.latest")
            self.tool_requires("autoconf/2.71")
            self.tool_requires("automake/1.16.5")

    def generate(self):
        if self.settings.os == "Windows":
            env = VirtualBuildEnv(self)
            env.generate()
        tc = AutotoolsToolchain(self)
        tc.generate()
        tc = PkgConfigDeps(self)

        env = Environment()
        compile_script = unix_path(self, self.dependencies.build["automake"].conf_info.get("user.automake:compile-wrapper"))
        # define CC and CXX such that if the user hasn't already defined it
        # via `tools.build:compiler_executables` or buildenv variables,
        # we tell autotools to guess the name matching the current setting
        # (otherwise it falls back to finding gcc first)
        cc = self._system_compiler()
        cxx = self._system_compiler(cxx=True)
        if cc and cxx:
            # Using shell parameter expansion
            env.define("CC", f"${{CC-{cc}}}")
            env.define("CXX", f"${{CXX-{cxx}}}")

        env.define("COMPILE", compile_script)
        env.define("ACLOCAL_PATH", unix_path(self, os.path.join(self.source_folder)))
        env.vars(self, scope="build").save_script("automakeconf")
