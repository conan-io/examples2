import platform

from test.examples_tools import chdir, run



if platform.system() == "Windows":

    # Build for Release and Debug with static libraries
    run("conan install . --output-folder=build --build=missing")
    with chdir("build"):
        command = []
        command.append("conanbuild.bat")
        command.append("cmake .. -G \"Visual Studio 17 2022\" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake")
        command.append("deactivate_conanbuild.bat")
        command.append("cmake --build . --config Release")
        cmd_out = run(" && ".join(command))
        assert "Building with CMake version: 3.22.6" in cmd_out
        cmd_out = run("Release\\compressor.exe")
        assert "ZLIB VERSION: 1.2.11" in cmd_out
else:
    run("conan install . --output-folder=cmake-build-release --build=missing")
    with chdir("cmake-build-release"):
        command = []
        command.append(". ./conanbuild.sh")
        command.append("cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
        command.append(". ./deactivate_conanbuild.sh")
        command.append("cmake --build .")
        cmd_out = run(" && ".join(command))
        assert "Building with CMake version: 3.22.6" in cmd_out
        cmd_out = run("./compressor")
        assert "ZLIB VERSION: 1.2.11" in cmd_out
