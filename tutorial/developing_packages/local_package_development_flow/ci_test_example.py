import platform
import os

from conan import conan_version

from test.examples_tools import run, chdir, replace


print("- Local developement flow -")


# FIXME: remove once 2.0-beta10 is out
prefix_preset_name = "" if "beta9" in str(conan_version) else "conan-"

run("conan source .")
run("conan install .")
run("conan build .")
run("conan export-pkg .")
cmd_out = run("conan list hello/1.0")
assert "hello/1.0" in cmd_out

# Using CMake directly, no conan build

if platform.system() == "Windows":
    run("rd /s /q src")
    run("rd /s /q build")
else:
    run("rm -rf src")
    run("rm -rf build")
    
run("conan remove hello/1.0 -c")

run("conan source .")
run("conan install .")

with chdir("src"):
    if platform.system() == "Windows":
        run(f"cmake --preset {prefix_preset_name}default")
        run(f"cmake --build --preset {prefix_preset_name}release")
    else:
        run(f"cmake --preset {prefix_preset_name}release")
        run(f"cmake --build --preset {prefix_preset_name}release")

run("conan export-pkg .")
cmd_out = run("conan list hello/1.0")
assert "hello/1.0" in cmd_out
