import os

from test.examples_tools import run, replace

print("- Declaring components in package_info() -")

out = run("conan create .")

assert "Conan: Component target declared 'algorithms'" in out
assert "Conan: Component target declared 'network'" in out
assert "Conan: Component target declared 'ai'" in out
assert "Conan: Component target declared 'rendering'" in out

replace(os.path.join("test_package", "CMakeLists.txt"), "algorithms", "nonexistent algorithms")

out = run("conan test test_package game-engine/1.0", error=True)

assert "Component 'nonexistent' NOT found in package 'game-engine'" in out
