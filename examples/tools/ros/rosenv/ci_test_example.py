import os
import platform
import subprocess

from test.examples_tools import chdir, run

test_dir_path = os.path.dirname(os.path.realpath(__file__))
if platform.system() == "Windows":
    test_dir_path = test_dir_path.replace("/", "\\")

docker_command = f"docker run --rm -v {test_dir_path}:/mnt/local osrf/ros:humble-desktop bash -c \"cd mnt/local && ./docker_commands.sh\""

run(docker_command)

# try:
#     # Setup the ROS environment
#     run("source /opt/ros/humble/setup.bash")

#     # Install the Conan dependencies of str_printer's package
#     with chdir("workspace"):
#         run("conan install str_printer/conanfile.txt --build=missing --output-folder install/conan")

#         # Setup the environment to find Conan installed dependencies
#         run("source install/conan/conanrosenv.sh")
#         # Perform the build
#         run("colcon build")

#         # Setup the run environment
#         run("source install/setup.bash")
#         # Run the consumer
#         run("ros2 run consumer main")
# finally:
#     # Remove all the bazel symlinks and clean its cache
#     shutil.rmtree("workspace/install", ignore_errors=True)
