from test.examples_tools import run

print("Pose estimation example with Tensorflow Lite and OpenCV")

run("conan install . -o opencv/\*:with_ffmpeg=False "
    "-o opencv/\*:with_gtk=False -c tools.system.package_manager:mode=install "
    "-c tools.system.package_manager:sudo=True")

run("cmake --preset conan-release")
run("cmake --build --preset conan-release")
run("build/Release/pose-estimation --no-windows")
