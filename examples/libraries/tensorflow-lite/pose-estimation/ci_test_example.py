import platform
from test.examples_tools import run

print("Pose estimation example with Tensorflow Lite and OpenCV")

# tensorflow-lite requires C++17, but MSVC defaults to C++14 (the other compilers default to gnu17)
cppstd = "-s compiler.cppstd=17 " if platform.system() == "Windows" else ""
install_cmd = "conan install . -c tools.system.package_manager:mode=install " \
              f"-c tools.system.package_manager:sudo=true {cppstd}--build=missing "

run(install_cmd)

# with presets

if platform.system() == "Windows":
    run("cmake --preset conan-default")
    run("cmake --build --preset conan-release")
    run("build\Release\pose-estimation.exe --no-windows --image=assets/dancing.png")
else:
    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")
    run("build/Release/pose-estimation --no-windows --image=assets/dancing.png")

if platform.system() == "Windows":
    run("rd /s /q build")
else:
    run("rm -rf build")

run(install_cmd)

# calling CMake directly

if platform.system() == "Windows":
    run("cmake . -G \"Visual Studio 17 2022\" -DCMAKE_TOOLCHAIN_FILE=./build/generators/conan_toolchain.cmake -DCMAKE_POLICY_DEFAULT_CMP0091=NEW")
    run("cmake --build . --config Release")
    run("Release\pose-estimation.exe --no-windows --image=assets/dancing.png")
else:
    run("cmake . -G \"Unix Makefiles\" -DCMAKE_TOOLCHAIN_FILE=build/Release/generators/conan_toolchain.cmake -DCMAKE_POLICY_DEFAULT_CMP0091=NEW -DCMAKE_BUILD_TYPE=Release")
    run("cmake --build .")
    run("./pose-estimation --no-windows --image=assets/dancing.png")
