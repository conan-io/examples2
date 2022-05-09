import platform

from test.examples_tools import run, tmp_dir

# ############# Example ################
print("- Use the CMakeToolchain to work locally (local flow) using CMakePresets feature. -")

output = run("cmake --version")
cmake_version = output.splitlines()[0].split(" ")[-1]
print(cmake_version)
if cmake_version < "3.23.1":
    print("SKIPPED TEST BECAUSE OF MISSING COMPATIBLE CMAKE")
    exit(0)

output = run("conan --version")
conan_version = output.splitlines()[0].split(" ")[-1]
print(conan_version)
if conan_version == "2.0.0-alpha6":
    print("SKIPPED TEST BECAUSE NOT COMPATIBLE WITH 2.0.0-alpha6")
    exit(0)


with tmp_dir("tmp"):
    run("conan new -d name=foo -d version=1.0 cmake_exe")
    run("conan install .")
    run("conan install . -s build_type=Debug")
    if platform.system() == "Windows":
        run("cmake --preset default")

        run("cmake --build --preset Release")
        output = run("build\\Release\\foo")
        print(output)
        assert "foo/1.0: Hello World Release!" in output

        run("cmake --build --preset Debug")
        output = run("build\\Debug\\foo")
        print(output)
        assert "foo/1.0: Hello World Debug!" in output

    else:
        run("cmake --preset Release")
        run("cmake --build --preset Release")
        output = run("./cmake-build-release/foo")
        print(output)
        assert "foo/1.0: Hello World Release!" in output

        run("cmake --preset Debug")
        run("cmake --build --preset Debug")
        output = run("./cmake-build-debug/foo")
        print(output)
        assert "foo/1.0: Hello World Debug!" in output
