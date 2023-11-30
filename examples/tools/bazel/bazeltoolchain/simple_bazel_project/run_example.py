import platform
import shutil

from test.examples_tools import run

# ############# Example ################
print("\n- Use the BazelToolchain and BazelDeps generators -\n")

if platform.system() in ("Windows", "Darwin"):
    print(f"SKIPPED TEST BECAUSE BAZEL IS NOT INSTALLED IN WINDOWS AND MACOS YET.")
    exit(0)

output = run("bazel --version")

try:
    if platform.system() == "Linux":
        print("\n- Conan installing all the files into the build folder -\n")
        run("conan install . --build=missing")
        print("\n- Setting up all the Meson variables declared by the conan_meson_native.ini file -\n")
        run("bazel --bazelrc=./conan/conan_bzl.rc build --config=conan-config //main:demo")
        print("\n- Running the application 'compressor' created -\n")
        run("./bazel-bin/main/demo")
finally:
    # Remove all the bazel symlinks and clean its cache
    run("bazel clean")
    shutil.rmtree("conan", ignore_errors=True)
