import os
import platform

from conan import conan_version

from test.examples_tools import run, chdir

print("- Workspaces and super builds -")

os.environ["CONAN_WORKSPACE_ENABLE"] = "will_break_next"

with chdir("mywksp"):
    run("conan workspace install")
    if platform.system() == "Windows":
        run("cmake --preset conan-default")
        run("cmake --build --preset conan-default")
        run(r"..\hello\build\Release\hello.exe")
    else:
        run("cmake --preset conan-default")
        run("cmake --build --preset conan-default")
        run("../hello/build/Release/hello")
    # Cleaning the build
    run(f"conan workspace clean")
