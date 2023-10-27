import platform
import os
import shutil

from test.examples_tools import chdir, run

def run_example(output_folder=""):

    if os.path.exists("build"):
        shutil.rmtree("build")

    run(f"conan install . {output_folder} --build missing")

    if platform.system() == "Windows":
        gen_folder = "" if output_folder else "generators\\"
        with chdir("build"):
            command = []
            command.append(f"cmake .. -G \"Visual Studio 17 2022\" -DCMAKE_TOOLCHAIN_FILE={gen_folder}conan_toolchain.cmake")
            command.append("cmake --build . --config Release")
            run(" && ".join(command))
            cmd_out = run("Release\\compressor.exe")
    else:
        build_path = "build" if output_folder else "build/Release"
        gen_folder = "" if output_folder else "generators/"
        cmakelists_path = ".." if output_folder else "../.."
        with chdir(build_path):
            command = []
            # in the conanfile.py we only add CMake as tool_require in Linux
            command.append(f". ./{gen_folder}conanbuild.sh")
            command.append(f"cmake {cmakelists_path} -DCMAKE_TOOLCHAIN_FILE={gen_folder}conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
            command.append("cmake --build .")
            command.append(f". ./{gen_folder}deactivate_conanbuild.sh")
            run(" && ".join(command))
            cmd_out = run("./compressor")

    assert "ZLIB VERSION: 1.2.11" in cmd_out


print("- Understanding the flexibility of using conanfile.py vs conanfile.txt -")

# first run the basic example without layout and conditionals 

run_example(output_folder="--output-folder=build")

# switch the conanfile's and code and run the more advanced example

os.rename("conanfile.py", "basic_conanfile.py")
os.rename(os.path.join("src", "main.c"), os.path.join("src", "basic_main.c"))
os.rename("CMakeLists.txt", "basic_CMakeLists.txt")
os.rename("complete_conanfile.py", "conanfile.py")
os.rename(os.path.join("src", "complete_main.c"), os.path.join("src", "main.c"))
os.rename("complete_CMakeLists.txt", "CMakeLists.txt")

run_example()

# restore original state
os.rename("conanfile.py", "complete_conanfile.py")
os.rename(os.path.join("src", "main.c"), os.path.join("src", "complete_main.c"))
os.rename("CMakeLists.txt", "complete_CMakeLists.txt")
os.rename("basic_conanfile.py", "conanfile.py")
os.rename(os.path.join("src", "basic_main.c"), os.path.join("src", "main.c"))
os.rename("basic_CMakeLists.txt", "CMakeLists.txt")
