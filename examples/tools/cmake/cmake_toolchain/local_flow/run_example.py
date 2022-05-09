import platform

from test.examples_tools import chdir, run, tmp_dir

# ############# Example ################
print("- Use the CMakeToolchain to work locally (local flow) -")

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
        with chdir("build"):
            run('cmake .. -G "Visual Studio 15 2017" -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake')
            run("cmake --build . --config Release")
            output = run("Release\\foo.exe")
            print(output)

            run('cmake .. -G "Visual Studio 15 2017" -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake')
            run("cmake --build . --config Debug")
            output = run("Debug\\foo.exe")
            print(output)
    else:
        with tmp_dir("cmake-build-release"):
            run("cmake ..  -DCMAKE_TOOLCHAIN_FILE=../build/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
            run("cmake --build .")
            output = run("./foo")
            print(output)
            assert "foo/1.0: Hello World Release!" in output

        with tmp_dir("cmake-build-debug"):
            run("cmake ..  -DCMAKE_TOOLCHAIN_FILE=../build/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug")
            run("cmake --build .")
            output = run("./foo")
            print(output)
            assert "foo/1.0: Hello World Debug!" in output
