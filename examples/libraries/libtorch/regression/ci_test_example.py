import platform
import shutil
import sys

from test.examples_tools import run, chdir

print("Libtorch regression example with Conan")

if platform.system() == "Windows":
    print("Skipping on Windows: libtorch dependencies not compatible with MSVC 195")
    sys.exit(0)

run("git clone --depth 1 https://github.com/pytorch/examples.git")
shutil.copy("conanfile.txt", "examples/cpp/regression/conanfile.txt")

with chdir("examples/cpp/regression"):
    cppstd = "17" if platform.system() == "Windows" else "gnu17"
    run(f"conan install -b=missing -s:a compiler.cppstd={cppstd} --update")

    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")

    # Only execute this in macos in the CI, there are some issues with the generated binary and the
    # GitHub images
    if platform.system() == "Darwin":
        run("./build/Release/regression")

