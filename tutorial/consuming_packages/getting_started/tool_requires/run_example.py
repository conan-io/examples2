import os
import platform
import subprocess
from contextlib import contextmanager


@contextmanager
def chdir(dir_path):
    current = os.getcwd()
    os.chdir(dir_path)
    try:
        yield
    finally:
        os.chdir(current)


def run(cmd, error=False):
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    out, err = process.communicate()
    out = out.decode("utf-8")
    err = err.decode("utf-8")
    ret = process.returncode

    output = err + out
    print("------------")
    print("Running: {}".format(cmd))
    print(output)
    print("------------")
    if ret != 0 and not error:
        raise Exception("Failed cmd: {}\n{}".format(cmd, output))
    if ret == 0 and error:
        raise Exception(
            "Cmd succeded (failure expected): {}\n{}".format(cmd, output))
    return output


run("rm -rf cmake-build-release")
run("conan install . --output-folder cmake-build-release --build missing")
run("cd cmake-build-release")
with chdir("cmake-build-release"):
    source_command = "" if platform.system() == "Windows" else ". ./"
    extension = ".bat" if platform.system() == "Windows" else ".sh"
    run(f"{source_command}conanbuild{extension} && cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake && cmake --build . && {source_command}deactivate_conanbuild{extension}")
    out = run("./compressor")
    assert "Built with CMake version: 3.19.8" in out
