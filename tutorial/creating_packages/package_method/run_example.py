import platform
from test.examples_tools import run

print("- Package method with cmake.install() -")

out = run(f"conan create . --build=missing")

print("- Package method with copy() -")

out = run(f"conan create manual_install.py --build=missing")
