import platform
import os

from test.examples_tools import chdir, run

def run_example(output_folder=""):

    run(f"conan install . {output_folder} --build missing")

    if platform.system() == "Windows":
        with chdir("build"):
            command = []
            command.append("cmake .. -G \"Visual Studio 15 2017\" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake")
            command.append("cmake --build . --config Release")
            run(" && ".join(command))
            cmd_out = run("Release\\compressor.exe")
    else:
        with chdir("build"):
            command = []
            # in the conanfile.py we only add CMake as tool_require in Linux
            command.append(". ./conanbuild.sh")
            command.append("cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
            command.append("cmake --build .")
            command.append(". ./deactivate_conanbuild.sh")
            run(" && ".join(command))
            cmd_out = run("./compressor")

    assert "ZLIB VERSION: 1.2.11" in cmd_out


print("- Understanding the flexibility of using conanfile.py vs conanfile.txt -")

# first run the basic example without layout and conditionals 

run_example(output_folder="--output-folder=build")

# switch the conanfile's and code and run the more advanced example

os.rename("conanfile.py", "basic_conanfile.py")
os.rename(os.path.join("src", "main.c"), os.path.join("src", "basic_main.c"))
os.rename("complete_conanfile.py", "conanfile.py")
os.rename(os.path.join("src", "complete_main.c"), os.path.join("src", "main.c"))

run_example()

os.rename("conanfile.py", "complete_conanfile.py")
os.rename(os.path.join("src", "main.c"), os.path.join("src", "complete_main.c"))
os.rename("basic_conanfile.py", "conanfile.py")
os.rename(os.path.join("src", "basic_main.c"), os.path.join("src", "main.c"))
