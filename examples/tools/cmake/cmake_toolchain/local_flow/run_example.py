import shutil
import subprocess
import platform
import os
import time

from contextlib import contextmanager


# FIXME: Extract this functions
def run(cmd, error=False, env_script=None, file_stdout=None):
    if env_script is not None:
        env = "{}.bat".format(env_script) if platform.system() == "Windows" else ". {}.sh".format(env_script)
        cmd = "{} && {}".format(env, cmd)
    # Used by tools/scm check_repo only (see if repo ok with status)
    print("Running: {}".format(cmd))
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    out, err = process.communicate()
    out = out.decode("utf-8")
    err = err.decode("utf-8")
    ret = process.returncode

    if file_stdout:
        open(file_stdout, "w").write(out)

    output = err + out
    if ret != 0 and not error:
        raise Exception("Failed cmd: {}\n{}".format(cmd, output))
    if ret == 0 and error:
        raise Exception("Cmd succeded (failure expected): {}\n{}".format(cmd, output))
    return output


@contextmanager
def tmp_dir(newdir):
    os.makedirs(newdir)
    try:
        with chdir(newdir):
            yield
    finally:
        shutil.rmtree(newdir)


@contextmanager
def chdir(newdir):
    old_path = os.getcwd()
    os.chdir(newdir)
    try:
        yield
    finally:
        os.chdir(old_path)


# ############# Example ################
print("- Use the CMakeToolchain to work locally (local flow) -")


with tmp_dir("tmp"):
    run("conan new -d name=foo -d version=1.0 cmake_exe")
    run("conan install .")
    run("conan install . -s build_type=Debug")
    if platform.system() == "Windows":
        with chdir("build"):
            run('cmake .. -G "Visual Studio 15 2017" -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake')
            run("cmake --build . --config Release")
            run("Release/foo.exe")

            run('cmake .. -G "Visual Studio 15 2017" -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake')
            run("cmake --build . --config Debug")
            run("Debug/foo.exe")
    else:
        with tmp_dir("cmake-build-release"):
            run("cmake ..  -DCMAKE_TOOLCHAIN_FILE=../build/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
            run("cmake --build .")
            output = run("./foo")
            print(output)
            assert "foo/1.0: Hello World Release!" in output

        with tmp_dir("cmake-build-debug"):
            run("cmake ..  -DCMAKE_TOOLCHAIN_FILE=../build/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug")
            run("cmake --build .")
            output = run("./foo")
            print(output)
            assert "foo/1.0: Hello World Debug!" in output
