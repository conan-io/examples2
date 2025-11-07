import platform
from test.examples_tools import run

print("Install xz_utils and validate SDK version")

if platform.system() == "Windows":
    run("conan install --requires=xz_utils/5.8.1 --build=xz_utils/5.8.1 -c:a tools.cmake.cmaketoolchain:extra_variables=\"{'CMAKE_POLICY_DEFAULT_CMP0149': 'NEW'}\"")
