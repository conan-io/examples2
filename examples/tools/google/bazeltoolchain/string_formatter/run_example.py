import platform
import shutil

from test.examples_tools import run

# ############# Example ################
print("\n- Use the BazelToolchain and BazelDeps generators -\n")

try:
    output = run("bazel --version")
except:
    print(f"SKIPPED TEST BECAUSE BAZEL IS NOT INSTALLED IN {platform.system()} PLATFORM YET.")
    exit(0)

try:
    print("\n- Conan installing all the files into the build folder -\n")
    run("conan install . --build=missing")
    print("\n- Running bazel build command to compile the demo bazel target -\n")
    run("bazel --bazelrc=./conan/conan_bzl.rc build --config=conan-config //main:demo")
    print("\n- Running the application 'demo' created -\n")
    run("./bazel-bin/main/demo")
finally:
    # Remove all the bazel symlinks and clean its cache
    run("bazel clean")
    shutil.rmtree("conan", ignore_errors=True)
