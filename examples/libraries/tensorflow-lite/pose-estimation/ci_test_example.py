import platform
from test.examples_tools import run

print("Pose estimation example with Tensorflow Lite and OpenCV")

install_cmd = "conan install . -c tools.system.package_manager:mode=install " \
              "-c tools.system.package_manager:sudo=true -s compiler.cppstd=17 --build=missing "

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
    run("cmake --preset conan-default")
    run("cmake --build --preset conan-release")
    run("build\Release\pose-estimation.exe --no-windows --image=assets/dancing.png")
else:
    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")
    run("build/Release/pose-estimation --no-windows --image=assets/dancing.png")
