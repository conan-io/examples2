import platform
from test.examples_tools import run

print("- Package method with cmake.install() -")

add_standard = "-s compiler.cppstd=11" if platform.system()!="Windows" else ""

out = run(f"conan create . {add_standard} --build=missing")

print("- Package method with copy() -")

out = run(f"conan create manual_install.py {add_standard} --build=missing")
