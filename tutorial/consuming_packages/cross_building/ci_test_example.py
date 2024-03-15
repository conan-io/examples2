import platform

from test.examples_tools import chdir, run

from conan import conan_version


if platform.system() == "Linux":

    run("conan install . --build missing -pr:b=default -pr:h=./profiles/raspberry")

    generators_folder = "Release/generators"

    with chdir("build"):
        command = []
        # in the conanfile.py we only add CMake as tool_require in Linux
        command.append(f". {generators_folder}/conanbuild.sh")
        command.append(f"cmake .. -DCMAKE_TOOLCHAIN_FILE={generators_folder}/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
        command.append("cmake --build .")
        command.append(f". {generators_folder}/deactivate_conanbuild.sh")
        run(" && ".join(command))
        cmd_out = run("file compressor")
        assert "ARM, EABI" in cmd_out
