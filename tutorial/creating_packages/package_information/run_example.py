import platform
import os

from test.examples_tools import run

print("- Define information for consumers depending on settings or options -")

out = run(f"conan create . --build=missing")

assertion = "Packaged 1 '.lib' file: hello-static.lib" if platform.system()=="Windows" else "Packaged 1 '.a' file: libhello-static.a"

assert assertion in out

print("- Properties model: setting information for specific generators -")

os.rename("conanfile.py", "conanfile_.py")
os.rename("conanfile_properties.py", "conanfile.py")

os.rename(os.path.join("test_package", "CMakeLists.txt"), os.path.join("test_package", "CMakeLists_.txt"))
os.rename(os.path.join("test_package", "CMakeLists_properties.txt"), os.path.join("test_package", "CMakeLists.txt"))

out = run(f"conan create . --build=missing")

assert "Target declared 'hello::myhello'" in out
