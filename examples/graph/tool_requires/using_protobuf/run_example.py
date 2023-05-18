import platform
import sys

from test.examples_tools import run

if platform.system() != "Darwin":
    print("Only MacOS platforms!")
    sys.exit()

# Running in native MacOS ARM
output = run("conan create myserver --build missing -pr:b myserver/apple-arch-armv8 -pr:h myserver/apple-arch-armv8")
assert "Test(): created a person with id 1337" in output
# Cross-building to Intel arch
output = run("conan create myserver --build missing -pr:b myserver/apple-arch-armv8 -pr:h myserver/apple-arch-x86_64")
assert "Test(): created a person with id 1337" not in output

# Assert the architectures for both testing examples created
output = run("file myserver/test_package/build/apple-clang-13.0-armv8-gnu17-release/example")
assert "64-bit executable arm64" in output
output = run("file myserver/test_package/build/apple-clang-13.0-x86_64-gnu17-release/example")
assert "64-bit executable x86_64" in output
