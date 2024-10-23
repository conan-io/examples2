
# Install pip and Conan
apt-get update && apt-get install -y python3 python3-pip
python3 -m pip install conan

# Setup the ROS environment
source /opt/ros/humble/setup.bash

# Setup Conan profile
conan profile detect

cd workspace
# Install the Conan dependencies of Consumer's package
#conan install consumer/conanfile.txt --build=missing --output-folder install/conan

# Setup the environment to find Conan installed dependencies
#source install/conan/conanrosenv.sh

# Perform the build
#colcon build

# Setup the run environment
source install/setup.bash
# Run the app
# ros2 run app main
