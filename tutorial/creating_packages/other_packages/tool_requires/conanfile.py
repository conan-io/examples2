import os
import stat
from conan import ConanFile
from conan.tools.files import save


class my_tool(ConanFile):
    name = "my_tool"
    version = "1.0"
    package_type = "application"

    settings = "os", "arch"

    def package(self):
        extension = "sh" if self.settings_build.os != "Windows" else "bat"
        app_path = os.path.join(self.package_folder, "bin", "say_hello.{}".format(extension))
        save(self, app_path, 'echo "Hello!!!"')
        os.chmod(app_path, os.stat(app_path).st_mode | stat.S_IEXEC)

    def package_info(self):
        self.buildenv_info.define("MYVAR", "23")
