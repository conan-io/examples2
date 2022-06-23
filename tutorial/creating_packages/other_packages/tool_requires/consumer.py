from conan import ConanFile


class MyConsumer(ConanFile):
    name = "my_library"
    version = "1.0"
    settings = "os", "arch", "compiler", "build_type"
    tool_requires = "my_tool/1.0"

    def build(self):
        if self.settings_build.os != "Windows":
            self.run("say_hello.sh")
            self.run("echo MYVAR=$MYVAR")
        else:
            self.run("say_hello.bat")
            self.run("echo MYVAR=%MYVAR%")
