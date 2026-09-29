import os
import platform
from test.examples_tools import run

print("Writing a Godot GDExtension in C++ with godot-cpp")

# godot-cpp requires C++17
run("conan build . -s compiler.cppstd=17 --build=missing")

lib = {"Windows": "libgdexample.template_debug.dll",
       "Darwin": "libgdexample.template_debug.dylib"}.get(platform.system(), "libgdexample.template_debug.so")
assert os.path.exists(os.path.join("demo", "bin", lib))
