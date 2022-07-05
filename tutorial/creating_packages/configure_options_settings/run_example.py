import platform
from test.examples_tools import run

print("- Configure settings and options in recipes -")

add_standard = "-s compiler.cppstd=11" if platform.system()!="Windows" else ""

out = run(f"conan create . --build=missing {add_standard} -s build_type=Release -o shared=True -o fPIC=True -tf=None")

assert "hello/1.0: Package" in out

out = run(f"conan create . --build=missing {add_standard} -s build_type=Release -o shared=True -o fPIC=False -tf=None")

assert "hello/1.0: Already installed!" in out
