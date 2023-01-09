import platform

from test.examples_tools import chdir, run


if platform.system() == "Linux":

    run("conan install . --build missing -pr:b=default -pr:h=./profiles/raspberry")

    with chdir("build"):
        command = []
        # in the conanfile.py we only add CMake as tool_require in Linux
        command.append(". generators/conanbuild.sh")
        command.append("cmake .. -DCMAKE_TOOLCHAIN_FILE=Release/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
        command.append("cmake --build .")
        command.append(". generators/deactivate_conanbuild.sh")
        run(" && ".join(command))
        cmd_out = run("file compressor")
        assert "ARM, EABI" in cmd_out
