import platform
from test.examples_tools import run

print("- Add requirements to packages -")

add_standard = "-s compiler.cppstd=11" if platform.system()!="Windows" else ""

run(f"conan create . --build=missing {add_standard}")
