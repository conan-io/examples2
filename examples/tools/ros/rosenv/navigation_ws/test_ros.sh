pipx ensurepath
source ~/.bashrc

source /opt/ros/kilted/setup.bash
rosdep init
rosdep update
rosdep install --from-paths navigation_package/
conan profile detect --force
conan install navigation_package/conanfile.txt --build=missing --output-folder install/conan
source install/conan/conanrosenv.sh
colcon build --packages-select navigation_package
source install/setup.bash
ros2 run navigation_package navigator navigation_package/locations.yaml
