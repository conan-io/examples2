import platform
import os
from test.examples_tools import run

print("An introduction to the Dear ImGui library")

run("conan install . -c tools.system.package_manager:mode=install "
    "-c tools.system.package_manager:sudo=True --build=missing")

# with presets

if platform.system() == "Windows":
    run("cmake --preset conan-default")
    run("cmake --build --preset conan-release")
else:
    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")

assert os.path.exists(os.path.join("build", "Release", "simple-shader.fs"))

if platform.system() == "Windows":
    run("rd /s /q build")
else:
    run("rm -rf build")

run("conan install . -c tools.system.package_manager:mode=install "
    "-c tools.system.package_manager:sudo=True")

# calling CMake directly

if platform.system() == "Windows":
    run("cmake . -G \"Visual Studio 17 2022\" -DCMAKE_TOOLCHAIN_FILE=./build/generators/conan_toolchain.cmake -DCMAKE_POLICY_DEFAULT_CMP0091=NEW")
    run("cmake --build . --config Release")
else:
    run("cmake . -G \"Unix Makefiles\" -DCMAKE_TOOLCHAIN_FILE=build/Release/generators/conan_toolchain.cmake -DCMAKE_POLICY_DEFAULT_CMP0091=NEW -DCMAKE_BUILD_TYPE=Release")
    run("cmake --build .")
