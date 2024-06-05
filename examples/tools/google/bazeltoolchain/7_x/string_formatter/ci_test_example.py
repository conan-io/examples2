import os
import platform
import shutil

from conan import conan_version

from test.examples_tools import run

if conan_version <= "2.3.2":
    print("SKIPPED TEST BECAUSE OF CONAN MINIMUM REQUIRED VERSION")
    exit(0)

# ############# Example ################
print("\n- Use the BazelToolchain and BazelDeps generators -\n")

path_mapping = {'Linux': '/usr/share/bazel-7.1.2/bin',
                'Windows': 'C:/bazel-7.1.2/bin',
                'Darwin': '/Users/jenkins/bazel-7.1.2/bin'}
# Add bazel path
os.environ["PATH"] += os.pathsep + path_mapping.get(platform.system())

output = run("bazel --version")

try:
    print("\n- Conan installing all the files into the build folder -\n")
    run("conan install . --build=missing")
    print("\n- Running bazel build command to compile the demo bazel target -\n")
    run("bazel --bazelrc=./conan/conan_bzl.rc build --config=conan-config //main:demo")
    print("\n- Running the application 'demo' created -\n")
    if platform.system() == "Windows":
        run("bazel-bin\\main\\demo.exe")
    else:
        run("./bazel-bin/main/demo")
finally:
    # Remove all the bazel symlinks and clean its cache
    run("bazel clean")
    shutil.rmtree("conan", ignore_errors=True)
