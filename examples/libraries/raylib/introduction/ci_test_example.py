import platform
from test.examples_tools import run

print("raylib example")

# not using a conanfile because that will be created by the CLion plugin, in case someone just wants to 
# copy this code to its folder so that the user does not find any conflicting file
run("conan install --requires=raylib/5.5 --build=missing -g CMakeDeps -g CMakeToolchain --output-folder=build")

# with presets

if platform.system() == "Windows":
    run("cmake --preset conan-default")
    run("cmake --build --preset conan-release")
else:
    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")
