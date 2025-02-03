from conan import ConanFile
from conan.tools.system import package_manager
from conan.tools.gnu import PkgConfig
from conan.errors import ConanInvalidConfiguration

required_conan_version = ">=2.0"


class SysNcursesConan(ConanFile):
    name = "ncurses"
    version = "system"
    description = "A textual user interfaces that work across a wide variety of terminals"
    topics = ("curses", "terminal", "toolkit")
    homepage = "https://invisible-mirror.net/archives/ncurses/"
    license = "MIT"
    package_type = "shared-library"
    settings = "os", "arch", "compiler", "build_type"

    def package_id(self):
        self.info.clear()

    def validate(self):
        supported_os = ["Linux", "Macos", "FreeBSD"]
        if self.settings.os not in supported_os:
            raise ConanInvalidConfiguration(f"{self.ref} wraps a system package only supported by {supported_os}.")

    def system_requirements(self):
        dnf = package_manager.Dnf(self)
        dnf.install(["ncurses-devel"], update=True, check=True)

        yum = package_manager.Yum(self)
        yum.install(["ncurses-devel"], update=True, check=True)

        apt = package_manager.Apt(self)
        apt.install(["libncurses-dev"], update=True, check=True)

        pacman = package_manager.PacMan(self)
        pacman.install(["ncurses"], update=True, check=True)

        zypper = package_manager.Zypper(self)
        zypper.install(["ncurses"], update=True, check=True)

        brew = package_manager.Brew(self)
        brew.install(["ncurses"], update=True, check=True)

        pkg = package_manager.Pkg(self)
        pkg.install(["ncurses"], update=True, check=True)

    def package_info(self):
        self.cpp_info.bindirs = []
        self.cpp_info.includedirs = []
        self.cpp_info.libdirs = []

        self.cpp_info.set_property("cmake_file_name", "Curses")
        self.cpp_info.set_property("cmake_target_name", "Curses::Curses")
        self.cpp_info.set_property("cmake_additional_variables_prefixes", ["CURSES",])

        pkg_config = PkgConfig(self, 'ncurses')
        pkg_config.fill_cpp_info(self.cpp_info, is_system=True)