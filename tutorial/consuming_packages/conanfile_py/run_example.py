import platform

from test.examples_tools import chdir, run


# following the cmake-layout declared in the conanfile.py
# it will install the conan generated files in 'build' folder
# for Windows and 'cmake-build-<configuration>' folder for Linux and Macos

run("conan install . --build missing")

if platform.system() == "Windows":
    with chdir("build"):
        command = []
        command.append("cmake .. -G \"Visual Studio 15 2017\" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake")
        command.append("cmake --build . --config Release")
        run(" && ".join(command))
        cmd_out = run("Release\\compressor.exe")
else:
    with chdir("cmake-build-release"):
        command = []
        # in the conanfile.py we only add CMake as tool_require in Linux
        command.append(". ./conanbuild.sh")
        command.append("cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
        command.append("cmake --build .")
        command.append(". ./deactivate_conanbuild.sh")
        run(" && ".join(command))
        cmd_out = run("./compressor")

assert "ZLIB VERSION: 1.2.11" in cmd_out
