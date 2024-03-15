import platform
from test.examples_tools import run

print("libcurl and stb example")

# not using a conanfile because that will be created by the CLion plugin, in case someone just wants to 
# copy this code to its folder so that the user does not find any conflicting file
run("conan install --requires=stb/cci.20220909 --build=missing -g CMakeDeps -g CMakeToolchain --output-folder=build")
run("conan install --requires=libcurl/8.2.0 --build=missing -g CMakeDeps -g CMakeToolchain --output-folder=build")

# with presets

if platform.system() == "Windows":
    run("cmake --preset conan-default")
    run("cmake --build --preset conan-release")
else:
    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")
