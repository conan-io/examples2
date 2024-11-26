source /opt/ros/humble/setup.bash
conan profile detect --force
conan install my_package/conanfile.txt --build=missing --output-folder install/conan
source install/conan/conanrosenv.sh
colcon build --packages-select my_package
source install/setup.bash
ros2 run my_package package my_package/locations.yaml
