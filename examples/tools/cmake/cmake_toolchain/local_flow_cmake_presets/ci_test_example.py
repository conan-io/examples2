import os
import platform

from test.examples_tools import run, tmp_dir

from conan import conan_version

# ############# Example ################
print("- Use the CMakeToolchain to work locally (local flow) using CMakePresets feature. -")

output = run("cmake --version")
cmake_version = output.splitlines()[0].split(" ")[-1]
print(cmake_version)
if cmake_version < "3.23.1":
    print("SKIPPED TEST BECAUSE OF MISSING COMPATIBLE CMAKE")
    exit(0)

# FIXME: remove once 2.0-beta10 is out
prefix_preset_name = "" if "beta9" in str(conan_version) else "conan-"

with tmp_dir("tmp"):
    run("conan new -d name=foo -d version=1.0 cmake_exe")
    run("conan install .")
    run("conan install . -s build_type=Debug")
    
    if platform.system() == "Windows":
        run(f"cmake --preset {prefix_preset_name}default")
    else:
        run(f"cmake --preset {prefix_preset_name}release")
        run(f"cmake --preset {prefix_preset_name}debug")
    
    run(f"cmake --build --preset {prefix_preset_name}release")
    output = run(str(os.path.join("build", "Release", "foo")))
    assert "foo/1.0: Hello World Release!" in output

    run(f"cmake --build --preset {prefix_preset_name}debug")
    output = run(str(os.path.join("build", "Debug", "foo")))
    assert "foo/1.0: Hello World Debug!" in output
