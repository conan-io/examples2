import platform
from test.examples_tools import run

print("raylib example")

run("conan install . --build=missing")

if platform.system() == "Windows":
    run("cmake --preset conan-default")
    run("cmake --build --preset conan-release")
else:
    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")
