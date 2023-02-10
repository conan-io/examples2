import platform
from test.examples_tools import run

print("- Configure settings and options in recipes -")

add_standard = "-s compiler.cppstd=11" if platform.system()!="Windows" else ""

ret_error = platform.system()=="Windows"

out = run(f"conan create . --build=missing {add_standard} -s build_type=Release -o shared=True -o fPIC=True", error=ret_error)

assertion = ("hello/1.0: Package" in out) if platform.system()!="Windows" else ("ERROR: option 'fPIC' doesn't exist" in out)

assert assertion

out = run(f"conan create . --build=missing {add_standard} -s build_type=Release -o shared=True -o fPIC=False", error=ret_error)

assertion = ("hello/1.0: Already installed!" in out) if platform.system()!="Windows" else ("ERROR: option 'fPIC' doesn't exist" in out)

assert assertion
