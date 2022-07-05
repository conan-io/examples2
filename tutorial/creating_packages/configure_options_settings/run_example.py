import platform
from test.examples_tools import run

print("- Configure settings and options in recipes -")

add_standard = "-s compiler.cppstd=11" if platform.system()!="Windows" else ""

out = run(f"conan create . --build=missing {add_standard} -s build_type=Release -tf=None")

out = run(f"conan create . --build=missing {add_standard} -s build_type=Debug -tf=None")
