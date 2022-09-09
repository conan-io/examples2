import platform
from test.examples_tools import run

print("- Building and running tests in the build() method -")

add_standard = "-s compiler.cppstd=11" if platform.system()!="Windows" else ""

out = run(f"conan create . {add_standard} --build=missing")

assert "Built target test_hello" in out
assert "Running 1 test from 1 test suite." in out

out = run(f"conan create . {add_standard} --build=missing -c tools.build:skip_test=True")

assert "Built target test_hello" not in out
assert "Running 1 test from 1 test suite." not in out
