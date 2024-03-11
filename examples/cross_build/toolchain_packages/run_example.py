import platform

from test.examples_tools import chdir, run


if platform.system() == "Linux":

    with chdir("toolchain"):
        run("conan create . -pr:b=default -pr:h=../profiles/raspberry-64 --build-require")
    
    with chdir("consumer"):
        run("conan install . --build missing -pr:b=default -pr:h=../profiles/raspberry-64 -pr:h=../profiles/arm-toolchain")

    generators_folder = "Release/generators"

    with chdir("consumer/build"):
        command = []
        # in the conanfile.py we only add CMake as tool_require in Linux
        command.append(f". {generators_folder}/conanbuild.sh")
        command.append(f"cmake .. -DCMAKE_TOOLCHAIN_FILE={generators_folder}/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
        command.append("cmake --build .")
        command.append(f". {generators_folder}/deactivate_conanbuild.sh")
        run(" && ".join(command))
        cmd_out = run("file compressor")
        assert "ELF 64-bit" in cmd_out

    run("rm -rf ./consumer/build")

    with chdir("consumer"):
        run("conan install . --build missing -pr:b=default -pr:h=../profiles/raspberry-64 -pr:h=../profiles/arm-toolchain")
        run("cmake --preset conan-release")
        run("cmake --build --preset conan-release")

    with chdir("consumer/build/Release"):
        cmd_out = run("file compressor")
        assert "ELF 64-bit" in cmd_out
