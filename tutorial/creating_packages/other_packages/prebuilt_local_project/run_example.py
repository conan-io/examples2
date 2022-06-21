import os

from test.examples_tools import run, chdir


# Let's generate and package a Release library
run("conan install . -s build_type=Release")
os.mkdir("build/Release")

with chdir("build/Release"):
    run("cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../generators/conan_toolchain.cmake")
    run("cmake --build .")

cmd_out = run("conan export-pkg . -s build_type=Release")

assert "Packaged 1 '.h' file: foo.h" in cmd_out
assert ("Packaged 1 '.a' file: libfoo.a" in cmd_out or "Packaged 1 '.lib' file: foo.lib" in cmd_out)

cmd_out = run("conan test test_package/conanfile.py foo/0.1 -s build_type=Release")
assert "foo/0.1: Hello World Release!" in cmd_out


# Let's generate and package a Debug library
run("conan install . -s build_type=Debug")
os.mkdir("build/Debug")

with chdir("build/Debug"):
    run("cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=../generators/conan_toolchain.cmake")
    run("cmake --build .")

cmd_out = run("conan export-pkg . -s build_type=Debug")

assert "Packaged 1 '.h' file: foo.h" in cmd_out
assert ("Packaged 1 '.a' file: libfoo.a" in cmd_out or "Packaged 1 '.lib' file: foo.lib" in cmd_out)

cmd_out = run("conan test test_package/conanfile.py foo/0.1 -s build_type=Debug")
assert "foo/0.1: Hello World Debug!" in cmd_out

run("rm -rf build")

# Using CMakePresets

run("conan install .")
run("cmake . --preset release")
run("cmake --build --preset release")

run("conan export-pkg .")
cmd_out = run("conan test test_package/conanfile.py foo/0.1")
assert "foo/0.1: Hello World Release!" in cmd_out

run("conan install . -s build_type=Debug")
run("cmake . --preset debug")
run("cmake --build --preset debug")

run("conan export-pkg . -s build_type=Debug")
cmd_out = run("conan test test_package/conanfile.py foo/0.1 -s build_type=Debug")
assert "foo/0.1: Hello World Debug!" in cmd_out


run("rm -rf build")
