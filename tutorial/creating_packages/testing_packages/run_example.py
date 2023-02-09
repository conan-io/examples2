from test.examples_tools import run

print("- Testing Conan packages: the test() method -")

out = run(f"conan create . --build=missing")

out = run(f"conan test test_package hello/1.0")

assert "hello/1.0: Hello World Release! (with color!)" in out
