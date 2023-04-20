import platform
from test.examples_tools import run

print("Pose estimation example with Tensorflow Lite and OpenCV")

run("conan install . -o opencv/\*:with_ffmpeg=False "
    "-o opencv/\*:with_gtk=False -c tools.system.package_manager:mode=install "
    "-c tools.system.package_manager:sudo=True -s compiler.cppstd=17")

# with presets

if platform.system() == "Windows":
    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")
    run("build/Release/pose-estimation --no-windows")
else:
    run("cmake --preset conan-default")
    run("cmake --build --preset conan-release")
    run("build\Release\pose-estimation.exe --no-windows")

if platform.system() == "Windows":
    run("rd /s /q build")
else:
    run("rm -rf build")

run("conan install . -o opencv/\*:with_ffmpeg=False "
    "-o opencv/\*:with_gtk=False -c tools.system.package_manager:mode=install "
    "-c tools.system.package_manager:sudo=True -s compiler.cppstd=17")

# calling CMake directly

if platform.system() == "Windows":
    run("cmake . -G \"Unix Makefiles\" -DCMAKE_TOOLCHAIN_FILE=build/Release/generators/conan_toolchain.cmake -DCMAKE_POLICY_DEFAULT_CMP0091=NEW -DCMAKE_BUILD_TYPE=Release")
    run("cmake --build .")
    run("./pose-estimation --no-windows")
else:
    run("cmake .. -G \"Visual Studio 15 2017\" -DCMAKE_TOOLCHAIN_FILE=./build/generators/conan_toolchain.cmake -DCMAKE_POLICY_DEFAULT_CMP0091=NEW'")
    run("cmake --build . --config Release")
    run("dir")
    run("pose-estimation.exe --no-windows")
