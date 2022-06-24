import platform
from test.examples_tools import run

print("- Preparing the build with in the generate() method -")

add_standard = "-s compiler.cppstd=11" if platform.system()!="Windows" else ""

out = run(f"conan create . --build=missing {add_standard} -o with_fmt=True")

assert "with color" in out

out = run(f"conan create . --build=missing -o with_fmt=False")

assert "without color" in out
