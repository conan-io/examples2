import platform
from test.examples_tools import run

print("- Add requirements to packages -")

run(f"conan create . --build=missing")
