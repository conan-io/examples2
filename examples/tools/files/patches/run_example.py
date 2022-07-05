from test.examples_tools import run

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

print("Patching with patch tool")
output = run("conan create conanfile_patch.py")
assert "hello/1.0: Hello Patched World Release!" in output

print("Patching with patch tool and conandata")
output = run("conan create conanfile_patch_conandata.py")
assert "hello/1.0: Hello Patched World Release!" in output

print("Patching with apply_conandata_patches tool")
output = run("conan create conanfile_apply_conandata.py")
assert "hello/1.0: Hello Patched World Release!" in output