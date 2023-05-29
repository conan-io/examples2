import platform
import sys

from test.examples_tools import run

if platform.system() != "Darwin":
    print("Only MacOS platforms!")
    sys.exit()

# Running in native MacOS ARM
output = run("conan create myaddresser --build missing -pr:b myaddresser/apple-arch-armv8 -pr:h myaddresser/apple-arch-armv8")
assert "myaddresser(): created a person with id 1337" in output
# Cross-building to Intel arch
output = run("conan create myaddresser --build missing -pr:b myaddresser/apple-arch-armv8 -pr:h myaddresser/apple-arch-x86_64")
assert "myaddresser(): created a person with id 1337" not in output

# Assert the architectures for both testing examples created
output = run("file myaddresser/test_package/build/apple-clang-13-armv8-gnu17-release/example")
assert "64-bit executable arm64" in output
output = run("file myaddresser/test_package/build/apple-clang-13-x86_64-gnu17-release/example")
assert "64-bit executable x86_64" in output
