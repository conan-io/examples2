import os

def run(cmd, error=False):
    ret = os.system(cmd)
    if ret != 0 and not error:
        raise Exception("Failed cmd: {}".format(cmd))
    if ret == 0 and error:
        raise Exception("Cmd succeded (failure expected): {}".format(cmd))


run("conan new cmake_lib -d name=hello -d version=2.0")
run("conan create .")
