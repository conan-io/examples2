import platform
from test.examples_tools import run

print("SDL2 getting started with Game Development and Conan")

ninja = "-c tools.cmake.cmaketoolchain:generator=Ninja " if platform.system() == "Windows" else ""
run(f"conan install . {ninja}-c tools.system.package_manager:mode=install "
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
    run("cmake --preset conan-default")
    run("cmake --build --preset conan-release")
else:
    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")
