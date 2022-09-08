import platform
from test.examples_tools import run

print("- Building and running tests in the build() method -")

out = run(f"conan create . --build=missing")

assert "Built target test_hello" in out
assert "Running 1 test from 1 test suite." in out

out = run(f"conan create . --build=missing -c tools.build:skip_test=True")

assert "Built target test_hello" not in out
assert "Running 1 test from 1 test suite." not in out
