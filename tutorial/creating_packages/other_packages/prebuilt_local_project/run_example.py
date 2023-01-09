import os
import platform

from test.examples_tools import run, chdir

run("conan new cmake_lib -d name=hello -d version=0.1 --force")

# Let's generate and package a Release library
run("conan install . -s build_type=Release")

os.makedirs("build/Release", exist_ok=True)

if platform.system() != "Windows":
    with chdir("build/Release"):
        run("cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../Release/generators/conan_toolchain.cmake")
        run("cmake --build .")
else:
    with chdir("build"):
        run("cmake .. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake")
        run("cmake --build . --config Release")

cmd_out = run("conan export-pkg . -s build_type=Release")

assert "Packaged 1 '.h' file: hello.h" in cmd_out
assert ("Packaged 1 '.a' file: libhello.a" in cmd_out or "Packaged 1 '.lib' file: hello.lib" in cmd_out)

cmd_out = run("conan test test_package/conanfile.py hello/0.1 -s build_type=Release")
assert "hello/0.1: Hello World Release!" in cmd_out


# Let's generate and package a Debug library
run("conan install . -s build_type=Debug")
os.mkdir("build/Debug")

if platform.system() != "Windows":
    with chdir("build/Debug"):
        run("cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=../Debug/generators/conan_toolchain.cmake")
        run("cmake --build .")
else:
    with chdir("build"):
        run("cmake .. -DCMAKE_TOOLCHAIN_FILE=Debug/generators/conan_toolchain.cmake")
        run("cmake --build . --config Debug")

cmd_out = run("conan export-pkg . -s build_type=Debug")

assert "Packaged 1 '.h' file: hello.h" in cmd_out
assert ("Packaged 1 '.a' file: libhello.a" in cmd_out or "Packaged 1 '.lib' file: hello.lib" in cmd_out)

cmd_out = run("conan test test_package/conanfile.py hello/0.1 -s build_type=Debug")
assert "hello/0.1: Hello World Debug!" in cmd_out


# Using CMakePresets, needed CMake > 3.23
if platform.system() == "Darwin":

    run("conan install .")
    run("cmake . --preset release")
    run("cmake --build --preset release")

    run("conan export-pkg .")
    cmd_out = run("conan test test_package/conanfile.py hello/0.1")
    assert "hello/0.1: Hello World Release!" in cmd_out

    run("conan install . -s build_type=Debug")
    run("cmake . --preset debug")
    run("cmake --build --preset debug")

    run("conan export-pkg . -s build_type=Debug")
    cmd_out = run("conan test test_package/conanfile.py hello/0.1 -s build_type=Debug")
    assert "hello/0.1: Hello World Debug!" in cmd_out



