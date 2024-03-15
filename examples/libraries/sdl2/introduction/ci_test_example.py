import platform
from test.examples_tools import run

print("SDL2 getting started with Game Development and Conan")

run("conan install . -c tools.system.package_manager:mode=install "
    "-c tools.system.package_manager:sudo=True --build=missing")

# with presets

if platform.system() == "Windows":
    run("cmake --preset conan-default")
    run("cmake --build --preset conan-release")
else:
    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")

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
