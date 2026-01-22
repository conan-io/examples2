import platform
import shutil

from test.examples_tools import run, chdir

print("Libtorch regression example with Conan")

run("git clone --depth 1 https://github.com/pytorch/examples.git")
shutil.copy("conanfile.txt", "examples/cpp/regression/conanfile.txt")

with chdir("examples/cpp/regression"):
    run("conan graph explain -s compiler.cppstd=17")
    # run("conan install -b=missing -s compiler.cppstd=17")
    #
    # if platform.system() == "Windows":
    #     run("cmake --preset conan-default")
    #     run("cmake --build --preset conan-release")
    # else:
    #     run("cmake --preset conan-release")
    #     run("cmake --build --preset conan-release")
    #
    # # Only execute this in macos in the CI, there are some issues with the generated binary and the
    # # GitHub images
    # if platform.system() == "Darwin":
    #     run("./build/Release/regression")

