import platform
from test.examples_tools import run

print("- Handle external sources -")


print("- Download sources from zip file -")

run("conan remove 'hello/1.0*' -f ")

add_standard = "-s compiler.cppstd=11" if platform.system()!="Windows" else ""

run(f"conan create . --build=missing {add_standard}")
