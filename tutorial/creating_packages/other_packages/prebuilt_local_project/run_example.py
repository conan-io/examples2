import os
import platform

from test.examples_tools import run, chdir

from conan import conan_version


run("conan new cmake_lib -d name=hello -d version=0.1 --force")

# Let's generate and package a Release library
run("conan install . -s build_type=Release")

os.makedirs("build/Release", exist_ok=True)

generators_folder_release = "Release/generators"
generators_folder_debug = "Debug/generators"

if platform.system() != "Windows":
    with chdir("build/Release"):
        run(f"cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../{generators_folder_release}/conan_toolchain.cmake")
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
os.makedirs("build/Debug", exist_ok=True)

if platform.system() != "Windows":
    with chdir("build/Debug"):
        run(f"cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=../{generators_folder_debug}/conan_toolchain.cmake")
        run("cmake --build .")
else:
    with chdir("build"):
        run("cmake .. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake")
        run("cmake --build . --config Debug")

cmd_out = run("conan export-pkg . -s build_type=Debug")

assert "Packaged 1 '.h' file: hello.h" in cmd_out
assert ("Packaged 1 '.a' file: libhello.a" in cmd_out or "Packaged 1 '.lib' file: hello.lib" in cmd_out)

cmd_out = run("conan test test_package/conanfile.py hello/0.1 -s build_type=Debug")
assert "hello/0.1: Hello World Debug!" in cmd_out

# FIXME: remove once 2.0-beta10 is out
prefix_preset_name = "" if "beta9" in str(conan_version) else "conan-"

# Using CMakePresets, needed CMake > 3.23
if platform.system() == "Darwin":

    run("conan install .")
    run(f"cmake . --preset {prefix_preset_name}release")
    run(f"cmake --build --preset {prefix_preset_name}release")

    run("conan export-pkg .")
    cmd_out = run("conan test test_package/conanfile.py hello/0.1")
    assert "hello/0.1: Hello World Release!" in cmd_out

    run("conan install . -s build_type=Debug")
    run(f"cmake . --preset {prefix_preset_name}debug")
    run(f"cmake --build --preset {prefix_preset_name}debug")

    run("conan export-pkg . -s build_type=Debug")
    cmd_out = run("conan test test_package/conanfile.py hello/0.1 -s build_type=Debug")
    assert "hello/0.1: Hello World Debug!" in cmd_out



