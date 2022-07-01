import os
import platform

from test.examples_tools import run, tmp_dir

# ############# Example ################
print("- Patch the sources with different methods -")


print("Patching with replace_in_file in the source() method")
output = run("conan create .")
assert "hello/1.0: Hello Friends! Release!" in output

print("Patching with replace_in_file in the build() method")
output = run("conan create conanfile_replace_build.py")
assert "hello/1.0: Hello Static Friends!" in output

output = run("conan create conanfile_replace_build.py -o shared=True")
assert "hello/1.0: Hello Shared Friends!" in output

