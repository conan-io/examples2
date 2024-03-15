import platform

from test.examples_tools import chdir, run


if platform.system() == "Windows":

    # Build for Release and Debug with static libraries
    run("conan install . --output-folder=build --build=missing -s build_type=Release")
    with chdir("build"):
        command = []
        command.append("conanbuild.bat")
        command.append("cmake .. -G \"Visual Studio 17 2022\" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake")
        command.append("deactivate_conanbuild.bat")
        command.append("cmake --build . --config Release")
        run(" && ".join(command))
        cmd_out = run("Release\\compressor.exe")
        assert "Release configuration!" in cmd_out

    run("conan install . --output-folder=build --build=missing -s build_type=Debug")
    with chdir("build"):
        command.append("conanbuild.bat")
        command.append("cmake .. -G \"Visual Studio 17 2022\" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake")
        command.append("deactivate_conanbuild.bat")
        command.append("cmake --build . --config Debug")
        run(" && ".join(command))
        cmd_out = run("Debug\\compressor.exe")
        assert "Debug configuration!" in cmd_out

    # Build for Release with shared libraries
    run("conan install . --output-folder=build --build=missing -s build_type=Release --options=zlib/1.2.11:shared=True")
    with chdir("build"):
        command = []
        command.append("conanbuild.bat")
        command.append("cmake .. -G \"Visual Studio 17 2022\" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake")
        command.append("deactivate_conanbuild.bat")
        command.append("cmake --build . --config Release")
        run(" && ".join(command))

        command = []
        command.append("conanrun.bat")
        command.append("Release\\compressor.exe")
        command.append("deactivate_conanrun.bat")
        cmd_out = run(" && ".join(command))
        assert "Release configuration!" in cmd_out

else:
    run("conan install . --output-folder=build --build=missing -s build_type=Release")
    with chdir("build"):
        command = []
        command.append(". ./conanbuild.sh")
        command.append("cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
        command.append(". ./deactivate_conanbuild.sh")
        command.append("cmake --build .")
        run(" && ".join(command))
        cmd_out = run("./compressor")
        assert "Release configuration!" in cmd_out

    run("conan install . --output-folder=build --build=missing -s build_type=Debug")
    with chdir("build"):
        command = []
        command.append(". ./conanbuild.sh")
        command.append("cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug")
        command.append(". ./deactivate_conanbuild.sh")
        command.append("cmake --build .")
        run(" && ".join(command))
        cmd_out = run("./compressor")
        assert "Debug configuration!" in cmd_out

    # Build for Release with shared libraries
    run("conan install . --output-folder=build --build=missing -s build_type=Release --options=zlib/1.2.11:shared=True")
    with chdir("build"):
        command = []
        command.append(". ./conanbuild.sh")
        command.append("cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
        command.append(". ./deactivate_conanbuild.sh")
        command.append("cmake --build .")
        run(" && ".join(command))

        command = []
        command.append(". ./conanrun.sh")
        command.append("./compressor")
        command.append(". ./deactivate_conanrun.sh")
        cmd_out = run(" && ".join(command))
        assert "Release configuration!" in cmd_out
