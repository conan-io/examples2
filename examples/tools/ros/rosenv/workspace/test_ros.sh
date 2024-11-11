source /opt/ros/humble/setup.bash
conan profile detect --force
cd examples/tools/ros/rosenv/workspace
conan install str_printer/conanfile.txt --build=missing --output-folder install/conan
source install/conan/conanrosenv.sh
colcon build --packages-select str_printer
colcon build --packages-select consumer
source install/setup.bash
ros2 run consumer main
rm -rf ./install
rm -rf ./log
rm -rf ./build
