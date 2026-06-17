import platform
import shutil

from test.examples_tools import run, chdir

print("Libtorch regression example with Conan")

run("git clone --depth 1 https://github.com/pytorch/examples.git")
shutil.copy("conanfile.txt", "examples/cpp/regression/conanfile.txt")

with chdir("examples/cpp/regression"):
    cppstd = "17" if platform.system() == "Windows" else "gnu17"
    ninja_conf = " -c tools.cmake.cmaketoolchain:generator=Ninja" if platform.system() == "Windows" else ""
    run(f"conan install -b=missing -s:a compiler.cppstd={cppstd}{ninja_conf} --update")

    run("cmake --preset conan-release")
    if platform.system() != "Windows":
        # Don't build on Windows. CI's msvc can't build this
        run("cmake --build --preset conan-release")

    # Only execute this in macos in the CI, there are some issues with the generated binary and the
    # GitHub images
    if platform.system() == "Darwin":
        run("./build/Release/regression")

