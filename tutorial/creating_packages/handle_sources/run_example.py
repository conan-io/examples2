from test.examples_tools import run, tmp_dir

print("- Handle external sources -")


print("- Download sources from zip file -")

run("conan remove 'hello*' -f ")

run("conan create .")
run("conan create . -s build_type=Debug")
run("conan create . -o hello/1.0:shared=True")

output = run("conan list recipes hello")

# check the package recipe is in the cache
assert "hello/1.0" in output

output = run("conan list packages hello/1.0#latest")

# assert that there are 3 binaries in the cache
assert output.count("hello/1.0#") == 3

print("- Clone sources from git repo -")

run("conan remove 'hello*' -f ")

run("conan create conanfile_git.py")
run("conan create conanfile_git.py -s build_type=Debug")
run("conan create conanfile_git.py -o hello/1.0:shared=True")

output = run("conan list recipes hello")

# check the package recipe is in the cache
assert "hello/1.0" in output

output = run("conan list packages hello/1.0#latest")

# assert that there are 3 binaries in the cache
assert output.count("hello/1.0#") == 3


print("- Download sources from zip file but using conandata.yml -")

run("conan remove 'hello*' -f ")

run("conan create conanfile_conandata.py")
output = run("conan list recipes hello")

# check the package recipe is in the cache
assert "hello/1.0" in output
