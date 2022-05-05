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
print("- Use the CMakeToolchain to work locally (local flow) using CMakePresets feature. -")

output = run("cmake --version")
cmake_version = output.splitlines()[0].split(" ")[-1]
print(cmake_version)
if cmake_version < "3.23.1":
    print("SKIPPED TEST BECAUSE OF MISSING COMPATIBLE CMAKE")
    exit(0)

output = run("conan --version")
conan_version = output.splitlines()[0].split(" ")[-1]
print(conan_version)
if conan_version == "2.0.0-alpha6":
    print("SKIPPED TEST BECAUSE NOT COMPATIBLE WITH 2.0.0-alpha6")
    exit(0)


with tmp_dir("tmp"):
    run("conan new -d name=foo -d version=1.0 cmake_exe")
    run("conan install .")
    run("conan install . -s build_type=Debug")
    if platform.system() == "Windows":
        run("cmake --preset default")

        run("cmake --build --preset Release")
        output = run("build\\Release\\foo")
        print(output)
        assert "foo/1.0: Hello World Release!" in output

        run("cmake --build --preset Debug")
        output = run("build\\Debug\\foo")
        print(output)
        assert "foo/1.0: Hello World Debug!" in output

    else:
        run("cmake --preset Release")
        run("cmake --build --preset Release")
        output = run("./cmake-build-release/foo")
        print(output)
        assert "foo/1.0: Hello World Release!" in output

        run("cmake --preset Debug")
        run("cmake --build --preset Debug")
        output = run("./cmake-build-debug/foo")
        print(output)
        assert "foo/1.0: Hello World Debug!" in output
