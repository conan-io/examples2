import os
import subprocess
import shutil
from contextlib import contextmanager


@contextmanager
def chdir(dir_path):
    current = os.getcwd()
    os.chdir(dir_path)
    try:
        yield
    finally:
        os.chdir(current)


@contextmanager
def tmp_dir(newdir):
    os.makedirs(newdir)
    try:
        with chdir(newdir):
            yield
    finally:
        shutil.rmtree(newdir)


def replace_in_file(filename, source, target):
    with open(filename, 'r') as file :
        filedata = file.read()

    filedata = filedata.replace(source, target)

    with open(filename, 'w') as file:
        file.write(filedata)


def run(cmd, error=False):
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    out, err = process.communicate()
    out = out.decode("utf-8")
    err = err.decode("utf-8")
    ret = process.returncode

    output = err + out
    print("Running: {}".format(cmd))
    print("----- OUTPUT -------")
    print(output)
    print("----END OUTPUT------")
    if ret != 0 and not error:
        raise Exception("Failed cmd: {}\n{}".format(cmd, output))
    if ret == 0 and error:
        raise Exception(
            "Cmd succeded (failure expected): {}\n{}".format(cmd, output))
    return output
