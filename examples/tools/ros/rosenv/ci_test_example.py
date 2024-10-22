import os
import platform
import shutil

from test.examples_tools import chdir, run


try:
    # Setup the ROS environment
    run("source /opt/ros/humble/setup.bash")

    # Install the Conan dependencies of Consumer's package
    with chdir("workspace"):
        run("conan install consumer/conanfile.txt --build=missing --install-path install/conan")

        # Setup the environment to find Conan installed dependencies
        run("source install/conan/conanrosenv.sh")
        # Perform the build
        run("colcon build")

        # Setup the run environment
        run("source install/setup.bash")
        # Run the app
        run("ros2 run app main")
finally:
    # Remove all the bazel symlinks and clean its cache
    shutil.rmtree("workspace/install", ignore_errors=True)
