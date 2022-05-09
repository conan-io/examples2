import platform

from examples_tools import chdir, run


# Run the example only in Linux where the cross-compile toolchain is available
# mocking the raspberry profile for the moment while we install the cross-compile toolchain
# also now working in alpha6 because of changes in cmake_layout

output = run("conan --version")
conan_version = output.splitlines()[0].split(" ")[-1]
print(conan_version)
if conan_version == "2.0.0-alpha6":
    print("SKIPPED TEST BECAUSE NOT COMPATIBLE WITH 2.0.0-alpha6")
    exit(0)

if platform.system() == "Linux":

    run("conan install . --build missing -pr:b=./profiles/ubuntu -pr:h=./profiles/raspberry")

    with chdir("build"):
        command = []
        # in the conanfile.py we only add CMake as tool_require in Linux
        command.append(". generators/conanbuild.sh")
        command.append("cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
        command.append("cmake --build .")
        command.append(". generators/deactivate_conanbuild.sh")
        run(" && ".join(command))
        cmd_out = run("file compressor")
        assert "ARM, EABI" in cmd_out
