import platform
from test.examples_tools import run

print("- Building and running tests in the build() method -")

out = run(f"conan create . -s compiler.cppstd=17 --build=missing --build=hello*")

assert "Running 1 test from 1 test suite." in out

out = run(f"conan create . -s compiler.cppstd=17 --build=missing --build=hello* -c tools.build:skip_test=True")

assert "Running 1 test from 1 test suite." not in out
