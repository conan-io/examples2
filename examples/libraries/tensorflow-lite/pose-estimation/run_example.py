from test.examples_tools import run

print("Pose estimation example with Tensorflow Lite and OpenCV")

run("conan create . --build=missing -c tools.system.package_manager:mode=install " \
    "-c tools.system.package_manager:sudo=True -o opencv/*:with_ffmpeg=False " \
    "-o opencv/*:with_gtk=False")