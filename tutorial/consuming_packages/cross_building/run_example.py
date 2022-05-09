import platform

from examples_tools import chdir, run


# Run the example only in Linux where the cross-compile toolchain is available
# mocking the raspberry profile for the moment while we install the cross-compile toolchain

if platform.system() == "Linux":

    run("conan install . --build missing -pr:b=./profiles/ubuntu -pr:h=./profiles/raspberry")

    with chdir("build"):
        run("cmake .. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug")
        run("cmake --build .")
