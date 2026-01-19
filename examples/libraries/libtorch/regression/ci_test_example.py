import platform
import shutil

from test.examples_tools import run, chdir

print("Libtorch regression example with Conan")


run("git clone https://github.com/pytorch/examples.git")
shutil.copy("conanfile.txt", "examples/cpp/regression/conanfile.txt")

with chdir("examples/cpp/regression"):
    run("conan install -b=missing")

    # with presets

    if platform.system() == "Windows":
        run("cmake --preset conan-default")
        run("cmake --build --preset conan-release")
        run("build/Release/regression.exe")
    else:
        run("cmake --preset conan-release")
        run("cmake --build --preset conan-release")
        run("./build/Release/regression")

