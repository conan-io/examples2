import platform
from test.examples_tools import run

print("- Package method with cmake.install() -")

out = run(f"conan create . --build=missing -s compiler.cppstd=17")

print("- Package method with copy() -")

out = run(f"conan create manual_install.py -s compiler.cppstd=17 --build=missing")
